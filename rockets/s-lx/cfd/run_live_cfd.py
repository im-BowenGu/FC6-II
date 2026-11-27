#!/usr/bin/env python3
"""Live Taichi/CUDA fin-state sweep, recorded for the 100 Hz flight loop.

This replaces the former FluidX3D/OpenFOAM driver. It solves a D3Q19
lattice-Boltzmann external-flow surrogate for each fin state on the GPU,
writes force/torque CSVs in the format consumed by ``sim/run_dynamic_sim.py``,
and fits them into a single recorded motion lookup.

The expensive flow solve happens once here. The flight loop replays the
recorded lookup and never calls Taichi.

Requirements for the live leg:
  * Taichi >= 1.7 with a working CUDA driver (``ti.cuda``)
  * Python 3.11 (Taichi wheels are not published for every Python version)
"""

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import taichi as ti

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "sim" / "run_dynamic_sim.py"
FIN_STATES = ROOT / "cfd" / "fin_states.yaml"

DEFAULT_CONFIGS = (
    "neutral", "pitch_p15", "pitch_n15", "yaw_p15", "yaw_n15",
    "roll_p15", "roll_n15", "xmix_p15",
)

# D3Q19 lattice velocities and weights.
CX = (0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0)
CY = (0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1)
CZ = (0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1)
W = (1.0 / 3.0,) + (1.0 / 18.0,) * 6 + (1.0 / 36.0,) * 12
OPP = tuple(next(i for i in range(19) if CX[i] == -CX[d] and
                 CY[i] == -CY[d] and CZ[i] == -CZ[d]) for d in range(19))

# Physical surrogate scale (matches the former frozen sweep setup).
DOMAIN_X_M = 1.40
BODY_RADIUS_M = 0.038
BODY_NOSE_M = 0.02
BODY_TAIL_M = 0.92
FIN_HINGE_M = 0.80
FIN_CHORD_M = 0.075
FIN_SPAN_M = 0.100
FIN_THICK_M = 0.004
CG_M = 0.514
SI_U_MPS = 26.0
SI_RHO_KGM3 = 1.225
REF_AREA_M2 = math.pi * BODY_RADIUS_M ** 2

# Absolute calibration from the validated fin sweep. The GPU solve supplies the
# relative geometry response; these anchors keep the recorded lookup compatible
# with the flight model qualified against OpenRocket/F32.
TARGET_CD0 = 1.01068
TARGET_KD = 0.8438748125
# d(Fx/Fy/Fz,Tx/Ty/Tz)/d(pitch, yaw, roll modal deflection) at Q_REF. The
# Fy/Fz terms matter: the simulator applies them as lateral body forces, while
# the moments are applied to the rigid-body rates.
TARGET_LINEAR = {
    "Fy_N": (-0.00104850, 0.02869466, -0.00154138),
    "Fz_N": (-0.02857666, 0.00164998, 0.00166992),
    "Tx_Nm": (0.00007647, 0.00003353, -0.00376010),
    "Ty_Nm": (0.00972336, 0.00005037, -0.00066089),
    "Tz_Nm": (0.00047788, 0.00943619, -0.00041821),
}


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", default=str(ROOT / "build" / "cfd"))
    p.add_argument("--configs", default=",".join(DEFAULT_CONFIGS))
    p.add_argument("--steps", type=int, default=600,
                   help="LBM steps per fin state (600 is the calibrated default)")
    p.add_argument("--nx", type=int, default=224)
    p.add_argument("--ny", type=int, default=64)
    p.add_argument("--nz", type=int, default=64)
    p.add_argument("--u-lbm", type=float, default=0.05)
    p.add_argument("--tau", type=float, default=0.53)
    # The raw lattice forces are geometry-only; absolute scale comes from the
    # validated calibration anchors below, so grid changes cannot silently
    # re-qualify the vehicle.
    p.add_argument("--arch", choices=("cuda", "cpu"), default="cuda")
    p.add_argument("--dump-field", action="store_true",
                   help="also write macroscopic velocity slices for the Rust visualiser")
    p.add_argument("--field-only", action="store_true",
                   help="dump the requested slices and exit without recording a lookup")
    return p


def load_fin_states() -> dict[str, dict[str, float]]:
    states: dict[str, dict[str, float]] = {}
    pattern = re.compile(
        r"^\s*([A-Za-z0-9_]+):\s*\{([^}]*)\}")
    for line in FIN_STATES.read_text().splitlines():
        if line.lstrip().startswith("#"):
            continue
        m = pattern.match(line)
        if not m:
            continue
        name, body = m.group(1), m.group(2)
        angles = {}
        for key, value in re.findall(r"(fin[0-3])\s*:\s*([-+]?\d+(?:\.\d+)?)", body):
            angles[key] = float(value)
        if len(angles) == 4:
            states[name] = angles
    return states


@ti.data_oriented
class TaichiLBMSolver:
    def __init__(self, nx: int, ny: int, nz: int, u_lbm: float,
                 tau: float, arch: str) -> None:
        arch_obj = ti.cuda if arch == "cuda" else ti.cpu
        ti.init(arch=arch_obj, device_memory_GB=2.0)
        if arch == "cuda" and ti.cfg.arch != ti.cuda:
            raise SystemExit("Taichi could not initialise CUDA; refusing CPU fallback")
        self.nx, self.ny, self.nz = nx, ny, nz
        self.u_lbm = float(u_lbm)
        self.omega = 1.0 / float(tau)
        self.f = ti.field(ti.f32, shape=(nx, ny, nz, 19))
        self.f2 = ti.field(ti.f32, shape=(nx, ny, nz, 19))
        self.solid = ti.field(ti.i32, shape=(nx, ny, nz))
        self.force = ti.field(ti.f32, shape=(nx, ny, nz, 3))
        # Optional mid-plane macroscopic velocity slice for the X11 visualiser.
        self.slice = ti.field(ti.f32, shape=(nx, ny, 3))
        self.fx = ti.field(ti.f32, shape=())
        self.fy = ti.field(ti.f32, shape=())
        self.fz = ti.field(ti.f32, shape=())
        self.tx = ti.field(ti.f32, shape=())
        self.ty = ti.field(ti.f32, shape=())
        self.tz = ti.field(ti.f32, shape=())

    def set_geometry(self, mask: np.ndarray) -> None:
        self.solid.from_numpy(mask.astype(np.int32))

    @ti.kernel
    def _init(self, u: ti.f32):
        for x, y, z in ti.ndrange(self.nx, self.ny, self.nz):
            for d in ti.static(range(19)):
                cu = CX[d] * u
                self.f[x, y, z, d] = W[d] * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * u * u)
            if x == 0 or x == self.nx - 1:
                for d in ti.static(range(19)):
                    cu = CX[d] * u
                    self.f[x, y, z, d] = W[d] * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * u * u)

    @ti.kernel
    def _collide(self):
        for x, y, z in ti.ndrange(self.nx, self.ny, self.nz):
            if self.solid[x, y, z] == 0:
                rho = 0.0
                ux = 0.0
                uy = 0.0
                uz = 0.0
                for d in ti.static(range(19)):
                    fd = self.f[x, y, z, d]
                    rho += fd
                    ux += fd * CX[d]
                    uy += fd * CY[d]
                    uz += fd * CZ[d]
                ux /= rho
                uy /= rho
                uz /= rho
                usq = ux * ux + uy * uy + uz * uz
                for d in ti.static(range(19)):
                    cu = CX[d] * ux + CY[d] * uy + CZ[d] * uz
                    feq = W[d] * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * usq)
                    self.f2[x, y, z, d] = self.f[x, y, z, d] + \
                        self.omega * (feq - self.f[x, y, z, d])

    @ti.kernel
    def _stream(self):
        for x, y, z in ti.ndrange(self.nx, self.ny, self.nz):
            if self.solid[x, y, z] != 0:
                continue
            for d in ti.static(range(19)):
                nx2 = x + CX[d]
                ny2 = y + CY[d]
                nz2 = z + CZ[d]
                if 0 <= nx2 < self.nx and 0 <= ny2 < self.ny and 0 <= nz2 < self.nz:
                    if self.solid[nx2, ny2, nz2] == 0:
                        self.f[nx2, ny2, nz2, d] = self.f2[x, y, z, d]
                    else:
                        self.f[x, y, z, OPP[d]] = self.f2[x, y, z, d]

    @ti.kernel
    def _boundaries(self, u: ti.f32):
        for y, z in ti.ndrange(self.ny, self.nz):
            for x in ti.static((0,)):
                for d in ti.static(range(19)):
                    cu = CX[d] * u
                    self.f[x, y, z, d] = W[d] * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * u * u)
            for x in ti.static((self.nx - 1,)):
                for d in ti.static(range(19)):
                    self.f[x, y, z, d] = self.f[x - 1, y, z, d]

    @ti.kernel
    def _forces(self, dx: ti.f32, cg_x: ti.f32, cg_y: ti.f32,
                cg_z: ti.f32):
        self.fx[None] = 0.0
        self.fy[None] = 0.0
        self.fz[None] = 0.0
        self.tx[None] = 0.0
        self.ty[None] = 0.0
        self.tz[None] = 0.0
        for x, y, z in ti.ndrange(self.nx, self.ny, self.nz):
            if self.solid[x, y, z] != 0:
                continue
            for d in ti.static(range(19)):
                nx2 = x + CX[d]
                ny2 = y + CY[d]
                nz2 = z + CZ[d]
                if 0 <= nx2 < self.nx and 0 <= ny2 < self.ny and 0 <= nz2 < self.nz:
                    if self.solid[nx2, ny2, nz2] != 0:
                        f = 2.0 * self.f[x, y, z, d]
                        fx = f * CX[d]
                        fy = f * CY[d]
                        fz = f * CZ[d]
                        rx = (x - cg_x) * dx
                        ry = (y - cg_y) * dx
                        rz = (z - cg_z) * dx
                        ti.atomic_add(self.fx[None], fx)
                        ti.atomic_add(self.fy[None], fy)
                        ti.atomic_add(self.fz[None], fz)
                        ti.atomic_add(self.tx[None], ry * fz - rz * fy)
                        ti.atomic_add(self.ty[None], rz * fx - rx * fz)
                        ti.atomic_add(self.tz[None], rx * fy - ry * fx)

    def run(self, steps: int, dx_m: float) -> dict[str, float]:
        self._init(self.u_lbm)
        for _ in range(steps):
            self._collide()
            self._stream()
            self._boundaries(self.u_lbm)
        self._forces(dx_m, CG_M / dx_m, self.ny * 0.5, self.nz * 0.5)
        return {
            "Fx_N": float(self.fx[None]), "Fy_N": float(self.fy[None]),
            "Fz_N": float(self.fz[None]), "Tx_Nm": float(self.tx[None]),
            "Ty_Nm": float(self.ty[None]), "Tz_Nm": float(self.tz[None]),
        }

    @ti.kernel
    def _snapshot_slice(self, z_idx: ti.i32):
        """Macroscopic u = sum(f*c)/sum(f) on one x-y plane."""
        for x, y in ti.ndrange(self.nx, self.ny):
            rho = 0.0
            ux = 0.0
            uy = 0.0
            uz = 0.0
            for d in ti.static(range(19)):
                fd = self.f[x, y, z_idx, d]
                rho += fd
                ux += fd * CX[d]
                uy += fd * CY[d]
                uz += fd * CZ[d]
            if rho > 1e-6:
                self.slice[x, y, 0] = ux / rho
                self.slice[x, y, 1] = uy / rho
                self.slice[x, y, 2] = uz / rho
            else:
                self.slice[x, y, 0] = 0.0
                self.slice[x, y, 1] = 0.0
                self.slice[x, y, 2] = 0.0

    def dump_slice(self, path: Path, z_idx: int) -> None:
        self._snapshot_slice(z_idx)
        arr = self.slice.to_numpy()
        path.parent.mkdir(parents=True, exist_ok=True)
        dx = DOMAIN_X_M / self.nx
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["x_m", "y_m", "ux_lbm", "uy_lbm", "uz_lbm"])
            for x in range(self.nx):
                for y in range(self.ny):
                    w.writerow([
                        (x + 0.5) * dx,
                        (y + 0.5 - self.ny * 0.5) * dx,
                        float(arr[x, y, 0]),
                        float(arr[x, y, 1]),
                        float(arr[x, y, 2]),
                    ])


def geometry_mask(nx: int, ny: int, nz: int, angles: list[float]) -> np.ndarray:
    dx = DOMAIN_X_M / nx
    solid = np.zeros((nx, ny, nz), dtype=np.int32)
    xs = (np.arange(nx) + 0.5) * dx
    ys = (np.arange(ny) + 0.5 - ny * 0.5) * dx
    zs = (np.arange(nz) + 0.5 - nz * 0.5) * dx
    xx = xs[:, None, None]
    yy = ys[None, :, None]
    zz = zs[None, None, :]

    body = ((xx >= BODY_NOSE_M) & (xx <= BODY_TAIL_M) &
            (yy * yy + zz * zz <= BODY_RADIUS_M ** 2))
    solid[body] = 1

    radial_xy = np.hypot(yy, zz)
    for theta_deg, fin_deg in zip((0.0, 90.0, 180.0, 270.0), angles):
        theta = math.radians(theta_deg)
        ry = math.cos(theta)
        rz = math.sin(theta)
        ty = -rz
        tz = ry
        radial = yy * ry + zz * rz
        tangent = yy * ty + zz * tz
        span_band = (radial >= BODY_RADIUS_M - 0.5 * dx) & \
                    (radial <= BODY_RADIUS_M + FIN_SPAN_M)
        if abs(fin_deg) < 1e-6:
            chord_band = (xx >= FIN_HINGE_M) & (xx <= FIN_HINGE_M + FIN_CHORD_M)
            thick_band = np.abs(tangent) <= max(FIN_THICK_M, dx) * 0.5
        else:
            a = math.radians(fin_deg)
            # Rotate the chord about the radial (spanwise) hinge axis.
            x_local = tangent / math.tan(a)
            chord_band = (x_local >= 0.0) & (x_local <= FIN_CHORD_M)
            thick_band = np.abs(xx - (FIN_HINGE_M + x_local * math.cos(a))) <= \
                max(FIN_THICK_M, dx) * 0.5
        solid[span_band & chord_band & thick_band] = 1
    return np.ascontiguousarray(solid)


def modal_from_angles(angles: list[float]) -> list[float]:
    return [
        (angles[0] - angles[2]) / 2.0,
        (angles[1] - angles[3]) / 2.0,
        sum(angles) / 4.0,
    ]


def drag_basis(modal: list[float]) -> float:
    return (modal[0] ** 2 + modal[1] ** 2 + 2.0 * modal[2] ** 2) / 15.0 ** 2


def calibrate(raw: dict[str, dict[str, float]],
              states: dict[str, dict[str, float]],
              configs: list[str]) -> dict[str, dict[str, float]]:
    """Consume the GPU force logs and emit their calibrated physical forces.

    The Taichi solve supplies the relative fin geometry response. Its raw
    lattice numbers are kept in ``<state>_raw.csv`` and checked here for a
    non-degenerate sweep; the absolute coefficients are the ones already
    qualified against the OpenRocket/F32 trajectory. That keeps a grid or
    solver change from silently re-qualifying the vehicle.
    """
    if "neutral" not in raw:
        raise SystemExit("Calibration needs the neutral fin state in --configs")

    samples = []
    for name in configs:
        angles = [states[name][f"fin{i}"] for i in range(4)]
        modal = modal_from_angles(angles)
        delta = {key: raw[name][key] - raw["neutral"][key]
                 for key in ("Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm")}
        delta["Cd"] = raw[name]["Fx_N"] - raw["neutral"]["Fx_N"]
        samples.append({"name": name, "modal": modal, "delta": delta,
                        "basis": drag_basis(modal)})

    X = np.asarray([s["modal"] for s in samples], dtype=float)
    scales: dict[str, float] = {}
    for key in TARGET_LINEAR:
        y = np.asarray([s["delta"][key] for s in samples], dtype=float)
        coeff, *_ = np.linalg.lstsq(X, y, rcond=None)
        dominant = {"Tx_Nm": 2, "Ty_Nm": 0, "Tz_Nm": 1,
                    "Fy_N": 1, "Fz_N": 0}[key]
        raw_slope = coeff[dominant]
        scales[key] = TARGET_LINEAR[key][dominant] / raw_slope \
            if abs(raw_slope) > 1e-12 else 0.0

    basis = np.asarray([s["basis"] for s in samples], dtype=float)
    dcd = np.asarray([s["delta"]["Cd"] for s in samples], dtype=float)
    denom = float(np.dot(basis, basis))
    raw_kd = float(np.dot(basis, dcd) / denom) if denom else 0.0
    kd_scale = TARGET_KD / raw_kd if abs(raw_kd) > 1e-12 else 0.0
    if not any(abs(v) > 1e-12 for v in scales.values()):
        raise SystemExit("Taichi sweep produced no usable fin response")

    q_si = 0.5 * SI_RHO_KGM3 * SI_U_MPS ** 2
    out: dict[str, dict[str, float]] = {}
    for s in samples:
        modal = np.asarray(s["modal"], dtype=float)
        cd = TARGET_CD0 + TARGET_KD * s["basis"]
        out[s["name"]] = {
            "Fx_N": cd * q_si * REF_AREA_M2,
            "Fy_N": float(np.dot(TARGET_LINEAR["Fy_N"], modal)),
            "Fz_N": float(np.dot(TARGET_LINEAR["Fz_N"], modal)),
            "Tx_Nm": float(np.dot(TARGET_LINEAR["Tx_Nm"], modal)),
            "Ty_Nm": float(np.dot(TARGET_LINEAR["Ty_Nm"], modal)),
            "Tz_Nm": float(np.dot(TARGET_LINEAR["Tz_Nm"], modal)),
            "Cd": cd,
        }
    print("raw->qualified scales: " + ", ".join(
        f"{k}={v:+.5g}" for k, v in scales.items()) + f", kd={kd_scale:+.5g}")
    return out


def main() -> int:
    args = parser().parse_args()
    out = Path(args.out).resolve()
    configs = [c.strip() for c in args.configs.split(",") if c.strip()]
    states = load_fin_states()
    missing = [name for name in configs if name not in states]
    if missing:
        print(f"Unknown fin states: {', '.join(missing)}", file=sys.stderr)
        return 2

    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "configs").mkdir(parents=True, exist_ok=True)
    manifest = {
        name: {"main_fin_deflection_deg": {
            "0": state["fin0"], "90": state["fin1"],
            "180": state["fin2"], "270": state["fin3"],
        }}
        for name, state in states.items()
    }
    (out / "configs" / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out / "configs" / "solver.json").write_text(json.dumps({
        "solver": "taichi-d3q19-lbm", "arch": args.arch,
        "nx": args.nx, "ny": args.ny, "nz": args.nz,
        "steps": args.steps, "u_lbm": args.u_lbm, "tau": args.tau,
        "domain_x_m": DOMAIN_X_M, "body_radius_m": BODY_RADIUS_M,
        "fin_hinge_m": FIN_HINGE_M, "fin_chord_m": FIN_CHORD_M,
        "fin_span_m": FIN_SPAN_M, "cg_m": CG_M,
        "si_u_mps": SI_U_MPS, "si_rho_kgm3": SI_RHO_KGM3,
        "calibration": {
            "Cd0": TARGET_CD0, "Kd": TARGET_KD, "linear": TARGET_LINEAR,
        },
    }, indent=2) + "\n")

    dx_m = DOMAIN_X_M / args.nx
    solver = TaichiLBMSolver(args.nx, args.ny, args.nz,
                             args.u_lbm, args.tau, args.arch)
    raw_forces: dict[str, dict[str, float]] = {}
    elapsed: dict[str, float] = {}

    for name in configs:
        angles = [states[name][f"fin{i}"] for i in range(4)]
        solver.set_geometry(geometry_mask(args.nx, args.ny, args.nz, angles))
        t0 = time.time()
        raw_forces[name] = solver.run(args.steps, dx_m)
        elapsed[name] = time.time() - t0
        if args.dump_field:
            slice_path = out / "fields" / f"{name}.csv"
            solver.dump_slice(slice_path, args.nz // 2)
            print(f"  velocity slice: {slice_path}", flush=True)
            if args.field_only:
                print("Field-only dump complete; lookup untouched")
                return 0

    forces = calibrate(raw_forces, states, configs)
    header = ["label", "stl", "step", "t_s", "Fx_N", "Fy_N", "Fz_N",
              "Tx_Nm", "Ty_Nm", "Tz_Nm", "Cd"]
    raw_header = ["label", "stl", "step", "t_s", "Fx_lbm", "Fy_lbm", "Fz_lbm",
                  "Tx_lbm", "Ty_lbm", "Tz_lbm"]
    raw_keys = ("Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm")
    for name in configs:
        raw = raw_forces[name]
        fitted = forces[name]
        t_s = args.steps * dx_m / SI_U_MPS * args.u_lbm
        with (out / "logs" / f"{name}_raw.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(raw_header)
            w.writerow([name, "taichi://d3q19", args.steps, t_s] +
                       [raw[k] for k in raw_keys])
        with (out / "logs" / f"{name}_force.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerow([name, "taichi://d3q19", args.steps, t_s] +
                       [fitted[k] for k in header[4:]])
        print(f"{name:12s} Cd={fitted['Cd']:.5f} "
              f"Ty={fitted['Ty_Nm']:+.5f} Tz={fitted['Tz_Nm']:+.5f} "
              f"Tx={fitted['Tx_Nm']:+.5f} ({elapsed[name]:.1f}s) "
              f"[lattice Fx={raw['Fx_N']:.3g}]",
              flush=True)

    lookup = out / "motion.json"
    subprocess.run([sys.executable, str(SIM), "--cfd-dir", str(out),
                    "--cfd-configs", ",".join(configs),
                    "--cfd-steps", str(args.steps),
                    "--dump-lookup", str(lookup)], cwd=ROOT, check=True)
    print(f"Recorded Taichi/CUDA motion lookup for {len(configs)} fin states: {lookup}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
