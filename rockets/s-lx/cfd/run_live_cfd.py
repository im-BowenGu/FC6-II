#!/usr/bin/env python3
"""Run a live FluidX3D sweep over selected fin movements, then record it.

This is the "CFD first, replay later" PR check:

1. Run FluidX3D (FP16S, D3Q19/SRT) headless for a small selection of fin
   states, one STL per state. No flow solver runs inside the flight loop.
2. Record the final force/torque/Cd of every state and fit them into a single
   motion lookup (``build/cfd/motion.json``) using the trajectory simulator's
   own fitter.
3. ``sim/run_dynamic_sim.py --check --motion build/cfd/motion.json`` then flies
   the 100 Hz firmware loop against that recorded lookup.

FluidX3D needs an OpenCL device. A GitHub-hosted runner has none, so CI for
this target must use a self-hosted runner labelled ``fluidx3d-opencl`` with a
FluidX3D checkout available (``FLUIDX3D_SRC``), or a prebuilt binary
(``--binary``). The committed ``sim/data/`` CSVs are the offline fallback for
local development; this script intentionally does not use them.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CFD_INPUTS = ROOT / "cfd" / "fluidx3d"
SIM = ROOT / "sim" / "run_dynamic_sim.py"
MANIFEST = ROOT / "sim" / "data" / "cfd" / "fluidx3d" / "configs" / "manifest.json"

DEFAULT_CONFIGS = (
    "neutral", "pitch_p15", "pitch_n15", "yaw_p15", "yaw_n15",
    "roll_p15", "roll_n15", "xmix_p15",
)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", default=os.environ.get("FLUIDX3D_SRC", ""),
                   help="FluidX3D source tree (contains src/ and makefile)")
    p.add_argument("--binary", default=os.environ.get("FLUIDX3D_BIN", ""),
                   help="prebuilt FluidX3D binary (implies --skip-build)")
    p.add_argument("--out", default=str(ROOT / "build" / "cfd"),
                   help="output directory for force logs and recorded lookup")
    p.add_argument("--configs", default=",".join(DEFAULT_CONFIGS),
                   help="comma-separated fin states to solve")
    p.add_argument("--steps", type=int, default=20000,
                   help="LBM time steps per state (20000 matches the frozen fit)")
    p.add_argument("--interval", type=int, default=400,
                   help="frame interval passed to setup.cpp")
    p.add_argument("--ocl-lib", default=os.environ.get("OCL_LIB", ""),
                   help="directory containing libOpenCL.so")
    p.add_argument("--icd", default=os.environ.get("ICD", ""),
                   help="OpenCL ICD vendor directory")
    p.add_argument("--skip-build", action="store_true",
                   help="reuse an already built FluidX3D binary")
    return p


def run(cmd: list[str], *, cwd: Path | None = None,
        env: dict[str, str] | None = None, log: Path | None = None) -> None:
    print("+ " + " ".join(cmd), flush=True)
    if log is None:
        subprocess.run(cmd, cwd=cwd, env=env, check=True)
    else:
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open("w") as f:
            subprocess.run(cmd, cwd=cwd, env=env, stdout=f,
                           stderr=subprocess.STDOUT, check=True)


def stl_for(name: str) -> Path:
    if name == "neutral":
        return CFD_INPUTS / "S-LX_cfd.stl"
    return CFD_INPUTS / "configs" / f"{name}.stl"


def main() -> int:
    args = parser().parse_args()
    out = Path(args.out).resolve()
    configs = [c.strip() for c in args.configs.split(",") if c.strip()]
    if not configs:
        print("No fin states selected", file=sys.stderr)
        return 2
    for name in configs:
        if not stl_for(name).is_file():
            print(f"Missing STL for fin state {name}: {stl_for(name)}",
                  file=sys.stderr)
            return 2

    source = Path(args.source).resolve() if args.source else None
    binary = Path(args.binary).resolve() if args.binary else None
    skip_build = args.skip_build or binary is not None
    if not skip_build:
        if source is None or not (source / "src").is_dir() or not (
                source / "makefile").is_file():
            print("FluidX3D source not found; set FLUIDX3D_SRC or pass "
                  "--source (needs src/ and makefile). Alternatively pass "
                  "--binary with --skip-build.", file=sys.stderr)
            return 2
        (source / "stl").mkdir(parents=True, exist_ok=True)
        shutil.copy2(CFD_INPUTS / "defines.hpp", source / "src" / "defines.hpp")
        shutil.copy2(CFD_INPUTS / "setup.cpp", source / "src" / "setup.cpp")
        for stl in [CFD_INPUTS / "S-LX_cfd.stl",
                    *sorted((CFD_INPUTS / "configs").glob("*.stl"))]:
            shutil.copy2(stl, source / "stl" / stl.name)
        make_cmd = ["make", "Linux"]
        if args.ocl_lib:
            make_cmd.append(f"LDLIBS_OPENCL=-L{args.ocl_lib} -lOpenCL")
        run(make_cmd, cwd=source, log=out / "logs" / "build.log")
        binary = source / "bin" / "FluidX3D"
    if binary is None:
        binary = (source / "bin" / "FluidX3D") if source else Path("FluidX3D")
    if not binary.is_file():
        print(f"FluidX3D binary not found: {binary}", file=sys.stderr)
        return 2

    (out / "logs").mkdir(parents=True, exist_ok=True)
    (out / "frames").mkdir(parents=True, exist_ok=True)
    (out / "configs").mkdir(parents=True, exist_ok=True)
    shutil.copy2(MANIFEST, out / "configs" / "manifest.json")

    cwd = source or binary.parent
    for name in configs:
        env = os.environ.copy()
        env.update({
            "SLX_STL": str(stl_for(name)),
            "SLX_T": str(args.steps),
            "SLX_STEP": str(args.interval),
            "SLX_LABEL": name,
            "SLX_OUT": str(out / "frames") + "/",
            "SLX_FORCELOG": str(out / "logs" / f"{name}_force.csv"),
        })
        if args.icd:
            env["OCL_ICD_VENDORS"] = args.icd
        if args.ocl_lib:
            env["LD_LIBRARY_PATH"] = args.ocl_lib
        run([str(binary), "0"], cwd=cwd, env=env,
            log=out / "logs" / f"{name}_run.log")
        force_log = out / "logs" / f"{name}_force.csv"
        if not force_log.is_file():
            print(f"FluidX3D wrote no force log for {name}", file=sys.stderr)
            return 2
        print(f"{name:12s} -> {force_log}", flush=True)

    lookup = out / "motion.json"
    run([sys.executable, str(SIM), "--cfd-dir", str(out),
         "--cfd-configs", ",".join(configs), "--cfd-steps", str(args.steps),
         "--dump-lookup", str(lookup)], cwd=ROOT)
    print(f"Recorded motion lookup for {len(configs)} fin states: {lookup}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
