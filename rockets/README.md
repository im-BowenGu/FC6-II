# Rocket profiles

This directory holds **per-rocket guidance profiles** that are compiled into
the upstream FC6-II firmware. It is additive: upstream targets and their build
behaviour are unchanged unless a profile is selected.

## Selecting a profile

```sh
# Build the full upstream tree with one extra target available
make -C src ROCKET=s-lx

# Build only the S-LX STM32F103 flight target
make -C src/s-lx-bluepill-f103 ROCKET=s-lx

# Local CI: host tests, model manifest and 100 Hz simulation
make -C rockets/s-lx ci

# Optional live-CFD leg (needs an OpenCL device and FLUIDX3D_SRC)
make -C rockets/s-lx ci-live

# Or run the standalone local CI script
rockets/s-lx/ci/run-local.sh
```

`src/Makefile.defs` includes `rockets/<ROCKET>/profile.mk` only when `ROCKET`
is set. That file contributes the profile's include path and source vpath; it
cannot remove or override any upstream source. `src/Makefile` appends the
profile's flight target to `ARMM3DIRS` only for the matching rocket.

## Adding a rocket

Create `rockets/<name>/` with:

| File | Purpose |
|---|---|
| `profile.mk` | `ROCKET_PROFILE_DIR`, `ROCKET_PROFILE_VPATH`, `ROCKET_PROFILE_CFLAGS` |
| `include/*.h`, `src/*.c` | hardware-independent guidance core and constants |
| `Makefile` | `test`, `sim-check`, `verify` targets |
| `model.yaml` | dimensions, motor, mass, aero inputs, target apogee and CAD references |
| `cad/` | rough model (OpenRocket `.ork`, STL geometry, fin/hinge geometry) |
| `cfd/` | movable-fin/canard state table and CFD roadmap |
| `sim/` | self-contained trajectory surrogate plus frozen reference data |
| `tests/` | host C tests and scenario assertions |

Keep the guidance core free of radio, GNSS, pyro and ground-command code. Rule
13 compliance is a per-profile property documented in the profile README.

## Verification contract

Local CI runs `make -C rockets/<name> ci` (equivalently `make test`), which
runs:

1. host C tests of the guidance constants, state machine and mixer;
2. a 100 Hz firmware-in-the-loop trajectory surrogate that asserts the target
   apogee within ±5 m and the duration window;
3. extended scenarios (sensor noise, 5° rail-exit offset and steady wind).

Every run writes per-mode telemetry, state-transition/event logs and summaries
to `rockets/<name>/sim/logs/` (override with `--log-dir`), including failed
checks. The surrogate is a regression oracle, not flight qualification.
Hardware-in-the-loop sensor, servo, SD and timing tests remain mandatory
before flight.

## Where CI runs

Verification is local-only. There is no GitHub Actions workflow and no pull
request gate. `rockets/s-lx/ci/run-local.sh` is the entry point for a
developer machine or a self-hosted local CI server; set `RUN_LIVE_CFD=1` to
include the FluidX3D fin-state sweep on a machine with an OpenCL device.
