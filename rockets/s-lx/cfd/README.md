# Movable-fin CFD: live sweep, recorded motion

This directory holds the self-contained FluidX3D inputs for the fin-state
sweep and the driver that runs them in the optional live-CFD leg of local CI.
The key distinction is **when** CFD runs:

* `run_live_cfd.py` solves a selection of static fin geometries once and
  records their forces/torques into one motion lookup.
* `../sim/run_dynamic_sim.py --motion build/cfd/motion.json` flies the real
  100 Hz firmware loop against that recorded lookup. The flight loop never
  calls the flow solver.

## What is in the live-CFD leg of local CI

`make -C rockets/s-lx ci-live` (or `make cfd-test`) performs:

1. `cfd/run_live_cfd.py` copies `defines.hpp`, `setup.cpp` and every fin STL
   into a FluidX3D source tree, builds the `Linux` target (unless
   `--skip-build`/`--binary`), and runs one headless LBM solve per fin state
   with `SLX_STL` selecting the geometry.
2. Final force/torque/Cd values are written to `build/cfd/logs/<state>_force.csv`.
   The same open-source fitter used by the simulator converts them into
   `build/cfd/motion.json` (dCd scale, lateral force and roll/pitch/yaw moment
   gradients). Only the 15° states are fitted; `xmix_p4`/`xmix_p8` stay out
   because a 4° plate is smaller than the 5.38 mm voxel scale.
3. `sim/run_dynamic_sim.py --check --motion build/cfd/motion.json` runs the
   OpenRocket F32 plant plus the recorded lookup at 100 Hz across eight
   scenarios and asserts apogee ±5 m, duration 34–37 s, 5° tilt recovery,
   noise and wind behaviour.

The committed CSVs in `../sim/data/cfd/fluidx3d/logs/` are the offline
fallback for `make ci`. The optional live leg does **not** use them: it
regenerates the lookup from a fresh sweep when `RUN_LIVE_CFD=1` is set.

## Fin states and geometry

One STL per state, all in `fluidx3d/`:

* `S-LX_cfd.stl` — neutral/body envelope;
* `configs/pitch_p15.stl`, `pitch_n15.stl`, `yaw_p15.stl`, `yaw_n15.stl`,
  `roll_p15.stl`, `roll_n15.stl`, `xmix_p15.stl` — default live-CI sweep;
* `configs/xmix_p4.stl`, `xmix_p8.stl` — validation-only reruns.

`fin_states.yaml` records the four fin angles for each case. FluidX3D uses
FP16S, D3Q19, SRT, 26 m/s freestream and an axis-aligned box around the
rocket; the force log is sampled from the final solver step.

## Running it locally

FluidX3D needs an OpenCL device, so this leg is optional and runs only on a
local machine with a usable OpenCL implementation. Set:

```sh
export FLUIDX3D_SRC=/path/to/FluidX3D   # source tree with src/ and makefile
export OCL_LIB=/path/to/opencl/lib      # only if libOpenCL is not on default path
export ICD=/path/to/OpenCL/vendors      # only if the ICD needs pinning
RUN_LIVE_CFD=1 rockets/s-lx/ci/run-local.sh
# or: make -C rockets/s-lx ci-live
```

For a quick smoke check without a full 8 × 20k-step sweep:

```sh
make -C rockets/s-lx cfd-record \
  CFD_ARGS="--configs neutral,pitch_p15 --steps 2000"
```

`--binary /path/to/FluidX3D --skip-build` reuses an existing build. Note that
short runs are not converged (the frozen fit uses 20k steps), so a partial
sweep is only useful for checking the plumbing, not the flight assertions.

## Roadmap

True moving-boundary CFD — re-meshing the fin (or an immersed boundary) every
control step and running the flow solver in lockstep with the motor burn —
remains a separate SDK/production task. The live sweep here is a static-state
surrogate: it is more current than the frozen CSVs but still not a
moving-boundary solve. It is suitable as a local regression oracle, not flight
qualification.

## Open hardware questions

* Confirm physical fin numbering: `xmix_p15` couples pitch and yaw rather
  than producing pure pitch.
* Measure hinge friction, servo lag and flutter on the bench; the simulator
  uses a first-order 35 ms lag and analytic damping.
