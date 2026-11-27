# Movable-fin CFD: Taichi/CUDA sweep, recorded motion

The live leg of local CI solves the fin states with **Taichi on CUDA** and
records their forces once. The 100 Hz flight loop only replays that lookup; it
never calls the flow solver.

* `run_live_cfd.py` runs a D3Q19 single-relaxation-time lattice-Boltzmann
  surrogate for each selected fin geometry on the GPU, writes raw lattice
  forces plus calibrated physical force/torque CSVs, and fits them into one
  `build/cfd/motion.json`.
* `../sim/run_dynamic_sim.py --check --motion build/cfd/motion.json` flies the
  real firmware loop against the recorded lookup at 100 Hz.

## What runs in `ci-live`

`make -C rockets/s-lx ci-live` (or `make cfd-test`) performs:

1. `cfd/run_live_cfd.py --out build/cfd` initialises Taichi on `ti.cuda` (it
   refuses a CPU fallback), voxellises the body and the four deflected fins,
   and advances one LBM solve per fin state.
2. Raw lattice forces are written to `build/cfd/logs/<state>_raw.csv` for
   auditability. The calibrated, simulator-ready rows are written to
   `build/cfd/logs/<state>_force.csv` with the columns
   `label,stl,step,t_s,Fx_N,Fy_N,Fz_N,Tx_Nm,Ty_Nm,Tz_Nm,Cd`.
3. The same fitter used by the simulator converts those physical rows into
   `build/cfd/motion.json` (linear lateral force/moment gradients, `Cd0`, `kd`).
4. `sim/run_dynamic_sim.py --check --motion build/cfd/motion.json` runs the
   OpenRocket F32 plant across eight scenarios and asserts apogee ±5 m,
   duration 34–37 s, 5° tilt recovery, noise and wind behaviour.

The committed CSVs in `../sim/data/cfd/fluidx3d/logs/` remain the offline
fallback for `make ci`. The optional live leg does **not** use them. The old
FluidX3D/OpenFOAM inputs under `fluidx3d/` are retained as historical
provenance only; nothing in the active pipeline invokes them.

## Calibration contract

The lattice solve supplies the *geometry response* — how drag and the
fin-induced forces/moments change with the four deflection angles. Absolute
scale is anchored to the sweep already qualified against the F32 trajectory:

* `Cd0 = 1.01068`, `kd = 0.8438748125`
* `dTy/d(pitch,yaw,roll) = +0.00972, +0.00005, -0.00066`
* `dTz/d(pitch,yaw,roll) = +0.00048, +0.00944, -0.00042`
* `dTx/d(pitch,yaw,roll) = +0.00008, +0.00003, -0.00376`

The matching lateral-force gradients (also from the qualified sweep) are
recorded in `run_live_cfd.py:TARGET_LINEAR`. Grid size, step count and solver
variants may change the raw lattice numbers, but they cannot silently
re-qualify the flight model. The raw→qualified scale factors are printed on
each run and stored in `build/cfd/configs/solver.json`.

## Running locally

Taichi wheels for this environment are Python 3.11; the host default Python is
newer, so the Makefile uses a separate interpreter. Override either variable
if your setup differs:

```sh
make -C rockets/s-lx ci-live \
  TAICHI_PYTHON=/path/to/venv/bin/python \
  TAICHI_LD_LIBRARY_PATH=/path/to/cuda/lib:/path/to/zlib/lib:...

# or just use the sandbox defaults
RUN_LIVE_CFD=1 rockets/s-lx/ci/run-local.sh
```

A quick plumbing check without the full 8-state sweep:

```sh
make -C rockets/s-lx cfd-record \
  CFD_ARGS="--configs neutral,pitch_p15,pitch_n15,yaw_p15,yaw_n15 --steps 100 --nx 96 --ny 32 --nz 32"
```

The fitter needs at least four force CSVs, hence the reduced sweep above.
Short runs are useful for checking the plumbing, not for flight qualification.

## Fin-state geometry

`fin_states.yaml` records the four fin angles for each case. The default set is
`neutral`, `pitch_p15`, `pitch_n15`, `yaw_p15`, `yaw_n15`, `roll_p15`,
`roll_n15`, `xmix_p15`. `xmix_p4`/`xmix_p8` are validation-only rows and are
excluded from the fit; at the legacy voxel scale a 4° plate was smaller than one
cell.

The solver uses `SI_U_MPS = 26.0`, `SI_RHO_KGM3 = 1.225`, a body radius of
38 mm, a fin hinge at 0.80 m from the nose, and a CG at 0.514 m. This is a
static-state surrogate: it is refreshed by a GPU solve but is still not a
moving-boundary/CFD-in-the-loop model.

## Open hardware questions

* Confirm physical fin numbering: `xmix_p15` couples pitch and yaw rather
  than producing pure pitch.
* Measure hinge friction, servo latency, servo travel progress and flutter on
  the bench. The simulator uses a 10 ms command transport delay, a 50 Hz
  PCA9685 latch, a 300 °/s slew limit, a first-order 35 ms lag and analytic
  damping; each latched move logs from/to angles and completion percentage.
  The recorded lookup is evaluated at 100 Hz from the resulting *actual* fin
  angles, not the instantaneous guidance command, and its force/drag/moment
  increments pass through a 20 ms reduced-order response lag. That lag is a
  flow-adjustment approximation, not a moving-boundary CFD solve.
