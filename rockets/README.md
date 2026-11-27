# Rocket profiles

This directory holds **per-rocket guidance profiles** that are compiled into
the upstream FC6-II firmware. It is additive: upstream targets and their build
behaviour are unchanged unless a profile is selected.

## Selecting a profile

```sh
# Build the full upstream tree with one extra target available
make -C src ROCKET=ukroc-2027

# Build only the UKROC STM32F103 flight target
make -C src/ukroc-bluepill-f103 ROCKET=ukroc-2027

# Host tests and PR verification for that rocket
make -C rockets/ukroc-2027 test
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

A pull request must pass `make -C rockets/<name> test`, which runs:

1. host C tests of the guidance constants, state machine and mixer;
2. a 100 Hz firmware-in-the-loop trajectory surrogate that asserts the target
   apogee within ±5 m and the duration window;
3. extended scenarios (sensor noise, 5° rail-exit offset and steady wind).

The surrogate is a regression oracle, not flight qualification. Hardware-in-
the-loop sensor, servo, SD and timing tests remain mandatory before flight.
