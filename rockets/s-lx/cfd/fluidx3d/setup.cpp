#include "setup.hpp"
#include <cstdlib>
#include <fstream>
#include <vector>

// Rough external-flow model of the S-LX rocket (UKROC 2027), run headless on the
// NVIDIA GPU through OpenCL. By default x = 0 at the nose tip, x = 0.920 m at
// the tail; freestream is +X (nose-first). Required defines.hpp extensions:
// FP16S, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, GRAPHICS. Built with
// "make Linux" (no X11).
//
// Environment variables (all optional):
//   SLX_STL      STL name under ../stl/, or an absolute/relative path
//   SLX_OUT      output directory (trailing '/' optional)
//   SLX_T        total LBM time steps (default 20000)
//   SLX_STEP     animation frame interval (default 400)
//   SLX_ANIM     "1" emits anim-<step>.png every SLX_STEP steps
//   SLX_CG       axial CG [m] from nose, used as torque origin (default 0.514)
//   SLX_FORCELOG CSV path for force/torque history
//   SLX_LABEL    run label recorded in the CSV and console log
// Geometry-only rendering (no flow solve, finer voxels, whole rocket framed):
//   SLX_GEOM=1   skip the LBM solve and write one geometry-only PNG
//   SLX_LX/SLX_LYZ/SLX_MEM  box length/width/VRAM MB (default 1.20/0.45/2500)
//   SLX_VIEW     iso|side|top (default iso)
//   SLX_ZOOM     camera fit factor; larger = tighter view (default 1.8 iso)
//   SLX_RX/SLX_RY  iso camera angles in degrees (default 70/20; 90/0 = side)

static string env_string(const char* const key, const string& fallback) {
	const char* v = std::getenv(key);
	return (v!=nullptr && v[0]!='\0') ? string(v) : fallback;
}

static bool env_flag(const char* const key) {
	const string v = env_string(key, "0");
	return v=="1" || v=="true" || v=="TRUE" || v=="yes" || v=="YES";
}

static ulong env_ulong(const char* const key, const ulong fallback) {
	const string v = env_string(key, "");
	if(v.empty()) return fallback;
	return (ulong)std::strtoull(v.c_str(), nullptr, 10);
}

static float env_float(const char* const key, const float fallback) {
	const string v = env_string(key, "");
	if(v.empty()) return fallback;
	return std::strtof(v.c_str(), nullptr);
}

static string with_slash(string path) {
	if(path.empty() || path.back()!='/') path += '/';
	return path;
}

void main_setup() {
	const bool geom = env_flag("SLX_GEOM");
	const string label = env_string("SLX_LABEL", "slx");
	const string view = env_string("SLX_VIEW", "iso");
	const float zoom = env_float("SLX_ZOOM", geom?1.8f:2.5f);
	const float cam_rx = env_float("SLX_RX", 70.0f);
	const float cam_ry = env_float("SLX_RY", 20.0f);

	// ################################## physical / numerical parameters ##################################
	// Geometry renders use a tighter box and more VRAM so the voxel size drops
	// from 5.38 mm to ~1.8 mm; the flow cases keep the original long domain.
	const float si_Lx = geom ? env_float("SLX_LX", 1.20f) : 2.20f;
	const float si_Ly = geom ? env_float("SLX_LYZ", 0.45f) : 1.00f;
	const float si_Lz = si_Ly;
	const float si_u = 26.0f;     // freestream speed [m/s] (approx peak airspeed from OpenRocket)
	const float si_rho = 1.225f;  // air density [kg/m^3]
	const float lbm_u = 0.05f;    // lattice freestream velocity
	const float lbm_Re = 4.0E5f;  // effective lattice Reynolds number (coarse-grid surrogate)
	const uint memory = geom ? (uint)env_ulong("SLX_MEM", 2500ull) : 900u;

	const uint3 lbm_N = resolution(float3(si_Lx, si_Ly, si_Lz), memory);
	units.set_m_kg_s((float)lbm_N.x, lbm_u, 1.0f, si_Lx, si_u, si_rho);
	const float lbm_nu = units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u);
	const float si_A = pif*sq(0.038f); // body frontal area [m^2] (76 mm max diameter)
	const ulong lbm_T = max(1ull, env_ulong("SLX_T", 20000ull));
	const ulong anim_step = max(1ull, env_ulong("SLX_STEP", 400ull));
	const bool anim = env_flag("SLX_ANIM");
	const float si_CG = env_float("SLX_CG", 0.514f); // OpenRocket CG, nose-referenced

	print_info("Grid = "+to_string(lbm_N.x)+"x"+to_string(lbm_N.y)+"x"+to_string(lbm_N.z));
	print_info("nu = "+to_string(lbm_nu, 8u)+", tau = "+to_string(3.0f*lbm_nu+0.5f, 8u));
	print_info("Cd reference area = "+to_string(si_A, 6u)+" m^2, mode = "+(geom?"geometry-only":"flow"));
	LBM lbm(lbm_N, lbm_nu);

	// ####################################### voxelise external rocket #######################################
	const float lbm_rocket_length = units.x(0.920f); // 920 mm overall
	// Flow cases place the rocket forwards in the long box to leave wake room;
	// geometry-only cases centre it for symmetric framing.
	const float3 center = geom ? lbm.center() : float3(0.42f*(float)lbm.get_Nx(), lbm.center().y, lbm.center().z);
	string stl_name = env_string("SLX_STL", "S-LX_cfd.stl");
	if(stl_name.find('/')==string::npos) stl_name = get_exe_path()+"../stl/"+stl_name;
	const string stl_path = create_file_extension(stl_name, ".stl");
	print_info("STL = "+stl_path+", cell = "+to_string(1000.0f*units.si_x(1.0f), 3u)+" mm");
	lbm.voxelize_stl(stl_path, center, lbm_rocket_length);

	// object_torque() uses position() in coordinates centred on the domain, so
	// subtract the box centre. The 0.920 m body is scaled to lbm_rocket_length.
	const float3 rotation_center = float3(center.x-lbm.center().x+(si_CG/0.920f-0.5f)*lbm_rocket_length,
	                                      0.0f, 0.0f);

	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.x[n] = lbm_u; // nose-first +X freestream
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // open box
	});

	// Headless renders have no default lights; without these the solid is almost black.
	set_light(0, float3(0.25f*(float)Nx, 0.80f*(float)Ny, 0.65f*(float)Nz));
	set_light(1, float3(0.65f*(float)Nx, 0.20f*(float)Ny, 0.35f*(float)Nz));
	const string out = with_slash(env_string("SLX_OUT",
		"/data/Projects/ASRC-SC60/rocket/cfd/fluidx3d/frames/"));

	// ###################################### geometry-only render ######################################
	if(geom) {
		lbm.run(2u, 2u); // two steps to initialise fields; no flow solve to speak of
		lbm.object_force(TYPE_S); // refresh the force field used for surface colouring
		lbm.graphics.visualization_modes = VIS_FLAG_SURFACE;
		if(view=="side")      lbm.graphics.set_camera_centered(90.0f, 0.0f, 60.0f, zoom);
		else if(view=="top")  lbm.graphics.set_camera_centered(0.0f, 90.0f, 60.0f, zoom);
		else                  lbm.graphics.set_camera_centered(cam_rx, cam_ry, 60.0f, zoom);
		lbm.graphics.write_frame(out, env_string("SLX_FRAME", "geom"), ".png");
		lbm.write_status(out);
		return;
	}

	// ###################################### run, visualise, measure ######################################
	const string log_path = env_string("SLX_FORCELOG", "");
	std::ofstream log;
	if(!log_path.empty()) {
		create_folder(log_path);
		log.open(log_path, std::ios::out|std::ios::trunc);
		log << "label,stl,step,t_s,Fx_N,Fy_N,Fz_N,Tx_Nm,Ty_Nm,Tz_Nm,Cd,Fx_lbm,Fy_lbm,Fz_lbm\n";
	}

	// Checkpoint schedule: animation mode samples regularly; otherwise use the
	// four canonical visualisation checkpoints.
	std::vector<ulong> stops;
	if(anim) {
		for(ulong t=anim_step; t<lbm_T; t+=anim_step) stops.push_back(t);
	} else {
		for(ulong t : {1000ull, 4000ull, 10000ull}) if(t<lbm_T) stops.push_back(t);
	}
	stops.push_back(lbm_T);

	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_FIELD;
	lbm.graphics.field_mode = 0; // velocity magnitude
	lbm.graphics.slice_mode = 2; // y slice -> x-z plane (side view)
	lbm.graphics.slice_y = (int)Ny/2;

	lbm.run(0u, lbm_T); // initialise
	ulong prev = 0ull;
	for(const ulong cp : stops) {
		lbm.run(cp-prev, lbm_T);
		prev = cp;

		const float3 F = lbm.object_force(TYPE_S);
		const float3 M = lbm.object_torque(rotation_center, TYPE_S);
		const float si_Fx = units.si_F(F.x), si_Fy = units.si_F(F.y), si_Fz = units.si_F(F.z);
		const float si_Tx = units.si_M(M.x), si_Ty = units.si_M(M.y), si_Tz = units.si_M(M.z);
		const float Cd = si_Fx/(0.5f*si_rho*sq(si_u)*si_A); // +X is downstream: +Fx is drag
		const string result = "SLX_RESULT label="+label+",step="+to_string(cp)
			+",t_s="+to_string(units.si_t(cp), 5u)
			+",Fx_N="+to_string(si_Fx, 5u)+",Fy_N="+to_string(si_Fy, 5u)+",Fz_N="+to_string(si_Fz, 5u)
			+",Tx_Nm="+to_string(si_Tx, 6u)+",Ty_Nm="+to_string(si_Ty, 6u)+",Tz_Nm="+to_string(si_Tz, 6u)
			+",Cd="+to_string(Cd, 4u);
		print_info(result);
		if(log.is_open()) {
			log << label << ',' << stl_path << ',' << cp << ',' << to_string(units.si_t(cp), 6u)
			    << ',' << to_string(si_Fx, 8u) << ',' << to_string(si_Fy, 8u) << ',' << to_string(si_Fz, 8u)
			    << ',' << to_string(si_Tx, 9u) << ',' << to_string(si_Ty, 9u) << ',' << to_string(si_Tz, 9u)
			    << ',' << to_string(Cd, 6u)
			    << ',' << to_string(F.x, 8u) << ',' << to_string(F.y, 8u) << ',' << to_string(F.z, 8u) << '\n';
			log.flush();
		}

		if(anim) {
			lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_FIELD;
			lbm.graphics.slice_mode = 2;
			lbm.graphics.set_camera_centered(90.0f, 0.0f, 60.0f, zoom);
			lbm.graphics.write_frame(out, "anim", ".png"); // anim-<step>.png
		} else {
			// Side elevation with a velocity field slice through the centreline.
			lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_FIELD;
			lbm.graphics.slice_mode = 2;
			lbm.graphics.set_camera_centered(90.0f, 0.0f, 60.0f, zoom);
			lbm.graphics.write_frame(out, "side", ".png");

			// Plan view with a velocity field slice through the centreline.
			lbm.graphics.slice_mode = 3;
			lbm.graphics.set_camera_centered(0.0f, 90.0f, 60.0f, zoom);
			lbm.graphics.write_frame(out, "top", ".png");
		}

		if(cp==lbm_T) {
			lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
			lbm.graphics.slice_mode = 0;
			lbm.graphics.set_camera_centered(cam_rx, cam_ry, 60.0f, zoom);
			lbm.graphics.write_frame(out, "wake_q", ".png");

			// Geometry-only views of the voxelised solid at the final step.
			lbm.graphics.visualization_modes = VIS_FLAG_SURFACE;
			lbm.graphics.set_camera_centered(90.0f, 0.0f, 60.0f, zoom);
			lbm.graphics.write_frame(out, "geometry_side", ".png");
			lbm.graphics.set_camera_centered(cam_rx, cam_ry, 60.0f, zoom);
			lbm.graphics.write_frame(out, "geometry_iso", ".png");
		}
	}
	if(log.is_open()) log.close();
	lbm.write_status(out);
} /**/
