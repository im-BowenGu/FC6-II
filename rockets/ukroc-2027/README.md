# UKROC 2027 guidance profile (additive upstream PR)

This is a **per-rocket profile** for the full upstream FC6-II tree. It is
compiled only when selected with `ROCKET=ukroc-2027`; no upstream file is
deleted and no existing target changes behaviour. The ESP32-C3 Rust flight
stack in the main rocket project is untouched.

## What is here

| File | Purpose |
|---|---|
| `profile.mk` | include/vpath glue consumed by `src/Makefile.defs` |
| `include/ukroc_profile.h` | flight constants, states, CSV schema, explicit no-radio/GPS/pyro rails |
| `src/ukroc_profile.c` | hardware-independent 100 Hz control law |
| `include/ukroc_stm32_pinmap.h` | STM32F103C8T6 Blue Pill pin map |
| `tests/test_ukroc_profile.c` | host tests for constants, interpolation, mixer, state machine, accel gate, coast weathervane, failsafe |
| `model.yaml` | rough model manifest: dimensions, motor, target, aero inputs, CAD references |
| `cad/`, `cfd/` | OpenRocket models, fin STL, movable-fin state table and roadmap |
| `sim/` | self-contained 100 Hz firmware-in-the-loop trajectory surrogate |
| `Makefile` | `make test` = offline host tests + PR checks; `make cfd-test` = live fin-state CFD + recorded-motion flight check |

The flight target lives at `src/ukroc-bluepill-f103/` and is appended to
`ARMM3DIRS` only for this rocket. It uses three new shared drivers
(`ao_bmp280`, `ao_icm20689`, `ao_pca9685`), all additive. The only shared-file
edits are: build glue gated on `ROCKET` in `src/Makefile`/`src/Makefile.defs`,
and a one-line default guard in `src/drivers/ao_fat.c` so a dedicated target
can set `FAT_COMMANDS=0`. No shared driver logic is rewritten.

## Flight profile

| Item | Value |
|---|---|
| Motor | AeroTech F32, 24 mm, single-use, 56.9 Ns, 4 s delay |
| Target apogee | 229 m / 750 ft, ±5 m in CI |
| Duration window | 34–37 s |
| Loop | 100 Hz, `DT = 0.01 s` |
| Launch detect | arm closed and `|a| > 2.0 g` |
| Rail hold | 300 ms, fins neutral |
| Boost schedule | 0° before 0.30 s; 1.9° until 2.20 s; 0° afterwards |
| Attitude filter | gyro integration with 3% accel correction, only when `az > 0.5 g` |
| Trim gains | `K_ALT = 0.010`, `K_VEL = 0.080`, `K_APOGEE = 0.200` |
| Apogee estimator | `APOGEE_DRAG_G = 1.55` |
| Trim clamp | 0–4° (can only reduce apogee) |
| Coast weathervane | after 2.0 s, centre fins if estimated body tilt > 3° |
| Servo clamp | ±15°, 1500 µs centre, 40 µs/deg |
| Descent | fins neutral at baro peak − 3 m |
| Fault | ten bad sensor reads or any init failure centres all fins |

CSV header (same as both flight stacks):

```
t_ms,state,p_pa,alt_m,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,fin0,fin1,fin2,fin3,vbat_v,vv_ms,pred_apo_m,ref_alt_m,ref_vv_ms,tilt_cmd_deg
```

## Rule 13 posture

No radio receiver/transmitter, no GNSS dependency, no ground-command parser
and no pyro output. Recovery is performed only by the F32 motor's own ejection
charge. The no-external-signal and no-pyro clauses need no clarification;
autonomous onboard control is explicitly permitted. The third sentence's
"commercially made altitude and/or timing devices" wording is a
hardware-provenance note — the IMU/barometer breakouts and MCU module are
commercial parts, while the assembled controller is specific to this rocket.

## Build and verify

```sh
make -C rockets/ukroc-2027 test     # offline: host C tests + frozen-lookup sim
make -C rockets/ukroc-2027 cfd-test # live CFD sweep, then recorded-motion sim
make -C src/ukroc-bluepill-f103 ROCKET=ukroc-2027   # ARM flight target
```

`cfd-test` runs FluidX3D once per selected fin state (eight states by
default, 20k steps each), writes the force/torque history to
`build/cfd/logs/`, fits it into `build/cfd/motion.json`, then runs the real
100 Hz firmware loop against **that recorded lookup**. The flight loop never
calls the flow solver. Requires an OpenCL device and `FLUIDX3D_SRC`, plus
`OCL_LIB`/`ICD` if the runner needs explicit ICD paths; a small smoke sweep
is `make cfd-record CFD_ARGS="--configs neutral,pitch_p15 --steps 2000"`.
The committed `sim/data/` CSVs remain the offline fallback for `make test`.

The GitHub Actions definition is stored as `ci/rocket-profile.yml` in this
profile directory. Copy it to `.github/workflows/rocket-profile.yml` in the
PR branch (needs a token with `workflow` scope; this fork's OAuth token does
not have it). The CFD job runs on a self-hosted `fluidx3d-opencl` runner
because GitHub-hosted runners have no OpenCL device; a separate Ubuntu job
fails the PR if any upstream file is deleted or `README.md` changes.

The simulator check prints all eight modes and asserts:

* nominal guided apogee **231.34 m / 35.50 s**, inside 229 ± 5 m and 34–37 s;
* 0.35 sensor noise: 231.43 m / 35.51 s;
* 5° rail-exit tilt: 228.97 m / 35.21 s, burnout error ≈0.98°, 1° band at
  0.84 s, coast fin ≤8.9°;
* steady 5 m/s wind: 232.36 m, no fin saturation.

These are regression bounds on the coupled FP16S/OpenRocket surrogate, **not**
flight qualification. There is no HIL bench here: real ICM-20689/BMP280/PCA9685
behaviour, SD writes, servo timing, the 100 Hz deadline, ejection and egg
survival remain unverified. The ARM build was checked with Newlib-nano/`nosys`
as a substitute for upstream `picolibc.specs`; upstream picolibc, flashing and
hardware timing still need verification.

## Known open items

* Verify physical fin numbering: `xmix_p15` couples pitch and yaw.
* Canopy `Cd ≈ 0.865` / `CdA = 0.150` needs a ground drop/ejection test.
* Flutter check and HIL sensor/servo/SD tests are outstanding.
* True moving-boundary CFD is a separate SDK/production task (`cfd/README.md`).
