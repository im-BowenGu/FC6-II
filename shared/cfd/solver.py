"""Config-driven D3Q19 lattice-Boltzmann flow surrogate."""

import math

import numpy as np
import taichi as ti

from .config import CfdConfig

CX = (0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0)
CY = (0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1)
CZ = (0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1)
W = (1.0 / 3.0,) + (1.0 / 18.0,) * 6 + (1.0 / 36.0,) * 12
OPP = tuple(next(i for i in range(19) if CX[i] == -CX[d] and
                 CY[i] == -CY[d] and CZ[i] == -CZ[d]) for d in range(19))


@ti.data_oriented
class TaichiLBMSolver:
    def __init__(self, config: CfdConfig, arch: str | None = None,
                 nx: int | None = None, ny: int | None = None,
                 nz: int | None = None, u_lbm: float | None = None,
                 tau: float | None = None) -> None:
        arch = config.solver.arch if arch is None else arch
        arch_obj = ti.cuda if arch == "cuda" else ti.cpu
        ti.init(arch=arch_obj, device_memory_GB=2.0)
        if arch == "cuda" and ti.cfg.arch != ti.cuda:
            raise SystemExit("Taichi could not initialise CUDA; refusing CPU fallback")
        self.config = config
        self.nx = config.solver.nx if nx is None else int(nx)
        self.ny = config.solver.ny if ny is None else int(ny)
        self.nz = config.solver.nz if nz is None else int(nz)
        self.u_lbm = float(config.solver.u_lbm if u_lbm is None else u_lbm)
        self.omega = 1.0 / float(config.solver.tau if tau is None else tau)
        self.f = ti.field(ti.f32, shape=(self.nx, self.ny, self.nz, 19))
        self.f2 = ti.field(ti.f32, shape=(self.nx, self.ny, self.nz, 19))
        self.solid = ti.field(ti.i32, shape=(self.nx, self.ny, self.nz))
        self.slice = ti.field(ti.f32, shape=(self.nx, self.ny, 3))
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

    def initialize(self) -> None:
        self._init(self.u_lbm)

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

    def step(self, n: int = 1) -> None:
        for _ in range(int(n)):
            self._collide()
            self._stream()
            self._boundaries(self.u_lbm)

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

    def forces(self, dx_m: float, cg_m: float) -> dict[str, float]:
        self._forces(dx_m, cg_m / dx_m, self.ny * 0.5, self.nz * 0.5)
        return {
            "Fx_N": float(self.fx[None]), "Fy_N": float(self.fy[None]),
            "Fz_N": float(self.fz[None]), "Tx_Nm": float(self.tx[None]),
            "Ty_Nm": float(self.ty[None]), "Tz_Nm": float(self.tz[None]),
        }

    def run(self, steps: int, dx_m: float) -> dict[str, float]:
        self.initialize()
        self.step(steps)
        return self.forces(dx_m, self.config.geometry.cg_m)

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

    def slice_array(self, z_idx: int) -> np.ndarray:
        self._snapshot_slice(int(z_idx))
        return np.array(self.slice.to_numpy(), copy=True)


def geometry_mask(config: CfdConfig, nx: int, ny: int, nz: int,
                  angles: list[float]) -> np.ndarray:
    g = config.geometry
    dx = g.domain_x_m / nx
    solid = np.zeros((nx, ny, nz), dtype=np.int32)
    xs = (np.arange(nx) + 0.5) * dx
    ys = (np.arange(ny) + 0.5 - ny * 0.5) * dx
    zs = (np.arange(nz) + 0.5 - nz * 0.5) * dx
    xx = xs[:, None, None]
    yy = ys[None, :, None]
    zz = zs[None, None, :]

    body = ((xx >= g.body_nose_m) & (xx <= g.body_tail_m) &
            (yy * yy + zz * zz <= g.body_radius_m ** 2))
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
        span_band = (radial >= g.body_radius_m - 0.5 * dx) & \
                    (radial <= g.body_radius_m + g.fin_span_m)
        if abs(fin_deg) < 1e-6:
            chord_band = (xx >= g.fin_hinge_m) & \
                         (xx <= g.fin_hinge_m + g.fin_chord_m)
            thick_band = np.abs(tangent) <= max(g.fin_thick_m, dx) * 0.5
        else:
            a = math.radians(fin_deg)
            x_local = tangent / math.tan(a)
            chord_band = (x_local >= 0.0) & (x_local <= g.fin_chord_m)
            thick_band = np.abs(xx - (g.fin_hinge_m + x_local * math.cos(a))) <= \
                max(g.fin_thick_m, dx) * 0.5
        solid[span_band & chord_band & thick_band] = 1
    return np.ascontiguousarray(solid)
