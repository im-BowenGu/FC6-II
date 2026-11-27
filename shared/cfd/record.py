"""Record a Taichi fin-state sweep as force CSVs and a motion lookup."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from .config import CfdConfig

FORCE_HEADER = ["label", "stl", "step", "t_s", "Fx_N", "Fy_N", "Fz_N",
                "Tx_Nm", "Ty_Nm", "Tz_Nm", "Cd"]
RAW_HEADER = ["label", "stl", "step", "t_s", "Fx_lbm", "Fy_lbm", "Fz_lbm",
              "Tx_lbm", "Ty_lbm", "Tz_lbm"]
RAW_KEYS = ("Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm")
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "rockets" / "s-lx" / "config" / "cfd.json"


def parser(config: CfdConfig | None = None) -> argparse.ArgumentParser:
    if config is None:
        config = CfdConfig.load(DEFAULT_CONFIG)
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=str(config.source))
    p.add_argument("--out", default=str(config.paths.out))
    p.add_argument("--configs", default=",".join(config.state_names(fit_only=True)))
    p.add_argument("--steps", type=int, default=config.solver.steps,
                   help="LBM steps per fin state")
    p.add_argument("--nx", type=int, default=config.solver.nx)
    p.add_argument("--ny", type=int, default=config.solver.ny)
    p.add_argument("--nz", type=int, default=config.solver.nz)
    p.add_argument("--u-lbm", type=float, default=config.solver.u_lbm)
    p.add_argument("--tau", type=float, default=config.solver.tau)
    p.add_argument("--arch", choices=("cuda", "cpu"), default=config.solver.arch)
    p.add_argument("--dump-field", action="store_true")
    p.add_argument("--field-only", action="store_true")
    return p


def _config_from_argv(argv: list[str]) -> str:
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=str(DEFAULT_CONFIG))
    known, _ = pre.parse_known_args(argv)
    return known.config


def _angles(states: dict[str, Any], name: str) -> list[float]:
    state = states[name]
    return [state.fin0, state.fin1, state.fin2, state.fin3]


def modal_from_angles(angles: list[float]) -> list[float]:
    return [
        (angles[0] - angles[2]) / 2.0,
        (angles[1] - angles[3]) / 2.0,
        sum(angles) / 4.0,
    ]


def drag_basis(modal: list[float]) -> float:
    return (modal[0] ** 2 + modal[1] ** 2 + 2.0 * modal[2] ** 2) / 15.0 ** 2


def calibrate(raw: dict[str, dict[str, float]],
              states: dict[str, Any],
              configs: list[str],
              config: CfdConfig) -> dict[str, dict[str, float]]:
    """Convert relative lattice forces into the qualified physical lookup."""
    if "neutral" not in raw:
        raise SystemExit("Calibration needs the neutral fin state in --configs")

    samples = []
    for name in configs:
        modal = modal_from_angles(_angles(states, name))
        delta = {key: raw[name][key] - raw["neutral"][key]
                 for key in ("Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm")}
        delta["Cd"] = raw[name]["Fx_N"] - raw["neutral"]["Fx_N"]
        samples.append({"name": name, "modal": modal, "delta": delta,
                        "basis": drag_basis(modal)})

    X = np.asarray([s["modal"] for s in samples], dtype=float)
    linear = config.calibration.linear
    scales: dict[str, float] = {}
    for key in linear:
        y = np.asarray([s["delta"][key] for s in samples], dtype=float)
        coeff, *_ = np.linalg.lstsq(X, y, rcond=None)
        dominant = {"Tx_Nm": 2, "Ty_Nm": 0, "Tz_Nm": 1,
                    "Fy_N": 1, "Fz_N": 0}[key]
        raw_slope = coeff[dominant]
        scales[key] = linear[key][dominant] / raw_slope \
            if abs(raw_slope) > 1e-12 else 0.0

    basis = np.asarray([s["basis"] for s in samples], dtype=float)
    dcd = np.asarray([s["delta"]["Cd"] for s in samples], dtype=float)
    denom = float(np.dot(basis, basis))
    raw_kd = float(np.dot(basis, dcd) / denom) if denom else 0.0
    kd_scale = config.calibration.kd / raw_kd if abs(raw_kd) > 1e-12 else 0.0
    if not any(abs(v) > 1e-12 for v in scales.values()):
        raise SystemExit("Taichi sweep produced no usable fin response")

    flow = config.flow
    q_si = 0.5 * flow.si_rho_kgm3 * flow.si_u_mps ** 2
    ref_area = math.pi * config.geometry.body_radius_m ** 2
    out: dict[str, dict[str, float]] = {}
    for s in samples:
        modal = np.asarray(s["modal"], dtype=float)
        cd = config.calibration.cd0 + config.calibration.kd * s["basis"]
        out[s["name"]] = {
            "Fx_N": cd * q_si * ref_area,
            "Fy_N": float(np.dot(linear["Fy_N"], modal)),
            "Fz_N": float(np.dot(linear["Fz_N"], modal)),
            "Tx_Nm": float(np.dot(linear["Tx_Nm"], modal)),
            "Ty_Nm": float(np.dot(linear["Ty_Nm"], modal)),
            "Tz_Nm": float(np.dot(linear["Tz_Nm"], modal)),
            "Cd": cd,
        }
    print("raw->qualified scales: " + ", ".join(
        f"{k}={v:+.5g}" for k, v in scales.items()) + f", kd={kd_scale:+.5g}")
    return out


def _write_slice(path: Path, array: np.ndarray, nx: int, ny: int,
                 dx_m: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["x_m", "y_m", "ux_lbm", "uy_lbm", "uz_lbm"])
        for x in range(nx):
            for y in range(ny):
                w.writerow([
                    (x + 0.5) * dx_m,
                    (y + 0.5 - ny * 0.5) * dx_m,
                    float(array[x, y, 0]),
                    float(array[x, y, 1]),
                    float(array[x, y, 2]),
                ])


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    config = CfdConfig.load(_config_from_argv(argv))
    args = parser(config).parse_args(argv)

    from .solver import TaichiLBMSolver, geometry_mask

    out = Path(args.out).resolve()
    configs = [c.strip() for c in args.configs.split(",") if c.strip()]
    states = config.states
    missing = [name for name in configs if name not in states]
    if missing:
        print(f"Unknown fin states: {', '.join(missing)}", file=sys.stderr)
        return 2

    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "configs").mkdir(parents=True, exist_ok=True)
    config.write_configs(
        out, arch=args.arch, nx=args.nx, ny=args.ny, nz=args.nz,
        steps=args.steps, u_lbm=args.u_lbm, tau=args.tau,
    )

    dx_m = config.geometry.domain_x_m / args.nx
    solver = TaichiLBMSolver(
        config, arch=args.arch, nx=args.nx, ny=args.ny, nz=args.nz,
        u_lbm=args.u_lbm, tau=args.tau,
    )
    raw_forces: dict[str, dict[str, float]] = {}
    elapsed: dict[str, float] = {}

    for name in configs:
        angles = _angles(states, name)
        solver.set_geometry(geometry_mask(config, args.nx, args.ny, args.nz, angles))
        t0 = time.time()
        raw_forces[name] = solver.run(args.steps, dx_m)
        elapsed[name] = time.time() - t0
        if args.dump_field:
            slice_path = out / "fields" / f"{name}.csv"
            _write_slice(slice_path, solver.slice_array(args.nz // 2),
                         args.nx, args.ny, dx_m)
            print(f"  velocity slice: {slice_path}", flush=True)
            if args.field_only:
                print("Field-only dump complete; lookup untouched")
                return 0

    forces = calibrate(raw_forces, states, configs, config)
    t_s = args.steps * dx_m / config.flow.si_u_mps * args.u_lbm
    for name in configs:
        raw = raw_forces[name]
        fitted = forces[name]
        with (out / "logs" / f"{name}_raw.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(RAW_HEADER)
            w.writerow([name, "taichi://d3q19", args.steps, t_s] +
                       [raw[k] for k in RAW_KEYS])
        with (out / "logs" / f"{name}_force.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FORCE_HEADER)
            w.writerow([name, "taichi://d3q19", args.steps, t_s] +
                       [fitted[k] for k in FORCE_HEADER[4:]])
        print(f"{name:12s} Cd={fitted['Cd']:.5f} "
              f"Ty={fitted['Ty_Nm']:+.5f} Tz={fitted['Tz_Nm']:+.5f} "
              f"Tx={fitted['Tx_Nm']:+.5f} ({elapsed[name]:.1f}s) "
              f"[lattice Fx={raw['Fx_N']:.3g}]",
              flush=True)

    lookup = out / "motion.json"
    subprocess.run([sys.executable, str(config.paths.sim),
                    "--cfd-dir", str(out),
                    "--cfd-configs", ",".join(configs),
                    "--cfd-steps", str(args.steps),
                    "--dump-lookup", str(lookup)], cwd=config.rocket_root, check=True)
    print(f"Recorded Taichi/CUDA motion lookup for {len(configs)} fin states: {lookup}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
