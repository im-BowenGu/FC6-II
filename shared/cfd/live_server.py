"""Live CFD field stream driven by the HIL telemetry CSV."""

from __future__ import annotations

import argparse
import csv
import io
import sys
import time
from pathlib import Path
from typing import Any

from .config import CfdConfig
from .stream import FieldStreamServer

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "rockets" / "s-lx" / "config" / "cfd.json"
ANGLE_TOLERANCE_DEG = 0.25


def read_telemetry(path: Path) -> dict[str, Any] | None:
    try:
        text = path.read_text(errors="replace")
    except FileNotFoundError:
        return None
    if text and not text.endswith(("\n", "\r")):
        text = text.rsplit("\n", 1)[0]
    if not text:
        return None
    rows = list(csv.reader(io.StringIO(text)))
    if not rows:
        return None
    header = rows[0]
    index = {name: i for i, name in enumerate(header)}
    fin_index = [index[f"fin{i}"] for i in range(4)]
    t_index = index.get("t_ms")
    state_index = index.get("state")
    for row in reversed(rows[1:]):
        if len(row) != len(header):
            continue
        try:
            angles = [float(row[i]) for i in fin_index]
            t_ms = float(row[t_index]) if t_index is not None else 0.0
        except (ValueError, TypeError):
            continue
        state = row[state_index].strip() if state_index is not None else ""
        return {"t_ms": t_ms, "angles": angles, "state": state}
    return None


def _config_from_argv(argv: list[str]) -> str:
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=str(DEFAULT_CONFIG))
    known, _ = pre.parse_known_args(argv)
    return known.config


def parser(config: CfdConfig | None = None) -> argparse.ArgumentParser:
    if config is None:
        config = CfdConfig.load(DEFAULT_CONFIG)
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=str(config.source))
    p.add_argument("--socket", default=None, help="default: build/cfd/field.sock")
    p.add_argument("--log", default=None, help="default: config.telemetry.csv")
    p.add_argument("--hz", type=float, default=8.0)
    p.add_argument("--steps-per-frame", type=int, default=6)
    p.add_argument("--arch", choices=("cuda", "cpu"), default=None)
    p.add_argument("--nx", type=int, default=None)
    p.add_argument("--ny", type=int, default=None)
    p.add_argument("--nz", type=int, default=None)
    return p


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    config = CfdConfig.load(_config_from_argv(argv))
    args = parser(config).parse_args(argv)
    if args.hz <= 0:
        parser(config).error("--hz must be positive")
    from .solver import TaichiLBMSolver, geometry_mask

    nx = config.solver.nx if args.nx is None else args.nx
    ny = config.solver.ny if args.ny is None else args.ny
    nz = config.solver.nz if args.nz is None else args.nz
    arch = config.solver.arch if args.arch is None else args.arch
    dx_m = config.geometry.domain_x_m / nx
    socket_path = config.rocket_root / "build/cfd/field.sock" \
        if args.socket is None else Path(args.socket).expanduser().resolve()
    log_path = config.telemetry.csv \
        if args.log is None else Path(args.log).expanduser().resolve()

    solver = TaichiLBMSolver(config, arch=arch, nx=nx, ny=ny, nz=nz)
    initial = read_telemetry(log_path)
    if initial is not None:
        angles = list(initial["angles"])
    elif "neutral" in config.states:
        angles = config.angles("neutral")
    else:
        angles = [0.0, 0.0, 0.0, 0.0]
    solver.set_geometry(geometry_mask(config, nx, ny, nz, angles))
    solver.initialize()
    active_angles = list(angles)

    seq = 0
    forces: dict[str, float] = {}
    started = time.monotonic()
    with FieldStreamServer(socket_path) as server:
        print(f"CFD stream {socket_path} {args.hz:g} Hz {nx}x{ny}x{nz}", flush=True)
        try:
            while True:
                tick = read_telemetry(log_path)
                current = list(tick["angles"]) if tick is not None else list(active_angles)
                if any(abs(a - b) > ANGLE_TOLERANCE_DEG
                       for a, b in zip(current, active_angles)):
                    solver.set_geometry(geometry_mask(config, nx, ny, nz, current))
                    active_angles = list(current)
                solver.step(args.steps_per_frame)
                seq += 1
                if (seq - 1) % 10 == 0:
                    forces = solver.forces(dx_m, config.geometry.cg_m)
                header = {
                    "seq": seq,
                    "t_ms": float(tick["t_ms"]) if tick is not None else 0.0,
                    "nx": nx,
                    "ny": ny,
                    "dx_m": dx_m,
                    "angles": current,
                    "forces": forces,
                    "state": tick["state"] if tick is not None else "",
                }
                server.publish(header, solver.slice_array(nz // 2))
                delay = started + seq / args.hz - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
