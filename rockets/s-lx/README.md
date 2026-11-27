# S-LX guidance profile (UKROC 2027 competition)

This is a **per-rocket profile** for the full upstream FC6-II tree. It is
compiled only when selected with `ROCKET=s-lx`; no upstream file is
deleted and no existing target changes behaviour. The ESP32-C3 Rust flight
stack in the main rocket project is untouched.

## What is here

| File | Purpose |
|---|---|
| `profile.mk` | include/vpath glue consumed by `src/Makefile.defs` |
| `include/slx_profile.h` | flight constants, states, CSV schema, explicit no-radio/GPS/pyro rails |
| `src/slx_profile.c` | hardware-independent 100 Hz control law |
| `include/slx_stm32_pinmap.h` | STM32F103C8T6 Blue Pill pin map |
| `tests/test_slx_profile.c` | host tests for constants, interpolation, mixer, state machine, accel gate, coast weathervane, failsafe |
| `model.yaml` | rough model manifest: dimensions, motor, target, aero inputs, CAD references |
| `config/` | per-rocket CFD, physics and visualiser configuration consumed by `shared/` |
| `cad/`, `cfd/` | OpenRocket models, fin STL, state table and compatibility CFD driver |
| `sim/` | self-contained 100 Hz firmware-in-the-loop trajectory surrogate |
| `../../shared/cfd/` | cross-rocket Taichi solver, force recorder and live field-stream server |
| `../../shared/physics/` | cross-rocket MuJoCo attitude engine |
| `../../shared/vis/rust/` | dependency-free Rust Xwayland window: live CFD, telemetry feed and rocket STL |
| `Makefile` | `make ci` = offline host tests + local CI checks; `make ci-live` = live fin-state CFD + recorded-motion replay |
| `ci/run-local.sh` | standalone local CI entry point; no GitHub Actions, no PR gate |

The flight target lives at `src/s-lx-bluepill-f103/` and is appended to
`ARMM3DIRS` only for this rocket. It uses three new shared drivers
(`ao_bmp280`, `ao_icm20689`, `ao_pca9685`), all additive. The only shared-file
edits are: build glue gated on `ROCKET` in `src/Makefile`/`src/Makefile.defs`,
and a one-line default guard in `src/drivers/ao_fat.c` so a dedicated target
can set `FAT_COMMANDS=0`. No shared driver logic is rewritten.

## Flight profile

| Item | Value |
|---|---|
| Motor | AeroTech F32, 24 mm, single-use, 56.9 Ns, 4 s delay |
| Target apogee | 229 m / 750 ft, ±5 m in local CI |
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
make -C rockets/s-lx ci       # local CI: host C tests + frozen-lookup sim
make -C rockets/s-lx ci-live  # live CFD sweep, then recorded-motion sim
rockets/s-lx/ci/run-local.sh  # standalone local CI entry point
make -C src/s-lx-bluepill-f103 ROCKET=s-lx   # ARM flight target
```

The GNU Make pipeline uses only tools available on a standard FHS Linux system
(`cc`, `python3`, `cargo`, `verilator`). Set `TAICHI_PYTHON` if Taichi and numpy
live in another interpreter, and `TAICHI_LD_LIBRARY_PATH` if the native Taichi
extension needs extra library directories. A parallel Nix pipeline is available
from the repository root:

```sh
nix develop            # FHS-equivalent toolchain without changing the Makefile
nix run .#slx-ci       # host tests + frozen-lookup sim
nix run .#slx-ci-live  # live Taichi/CUDA sweep + replay
nix run .#slx-vis      # build the shared Rust visualiser
```

`ci-live` still needs a Taichi/CUDA Python: set `TAICHI_PYTHON` and
`TAICHI_LD_LIBRARY_PATH` before either CI entry point. The Nix scripts copy the
profile and `shared/` into a writable temporary tree, so no build output is
written into the Nix store.

Every simulation run writes `sim/logs/<mode>.csv` (100 Hz telemetry),
`sim/logs/<mode>.events.jsonl` (state transitions and flight milestones) and
`sim/logs/summaries.json` / `run.json`, including when `--check` fails. Use
`python3 sim/run_dynamic_sim.py --check --log-dir /path/to/logs` to redirect
them. Logs are intentionally ignored by git. `make -C rockets/s-lx/hil run`
writes HIL logs to `hil/logs/cfd`, which is what the GUI's default `cfd` run
selector reads.

The live CFD leg is GPU-bound: during it, watch `nvidia-smi dmon -s u -d 1`
rather than CPU usage. `cfd-test` runs a Taichi D3Q19 lattice-Boltzmann solve
once per selected fin state on CUDA (eight states, 600 steps each by default),
writes raw lattice forces and calibrated physical force/torque rows to
`build/cfd/logs/`, fits
them into `build/cfd/motion.json`, then runs the real 100 Hz firmware loop
against **that recorded lookup**. The flight loop never calls the flow solver.
It requires a CUDA-capable GPU and Taichi 1.7+ in a Python 3.11 interpreter;
override `TAICHI_PYTHON` and `TAICHI_LD_LIBRARY_PATH` if needed. The committed
`sim/data/` CSVs remain the offline fallback for
`make test`; the old FluidX3D/OpenFOAM inputs are retained as provenance only.

There is no GitHub Actions workflow and no PR gate. `ci/run-local.sh` runs
offline by default and adds the live-CFD leg only when `RUN_LIVE_CFD=1` on a
machine with Taichi and a CUDA GPU.

The simulator check prints all eight modes and asserts:

* nominal guided apogee **231.01 m / 35.45 s**, inside 229 ± 5 m and 34–37 s;
* 0.35 sensor noise: 231.14 m / 35.47 s;
* 5° rail-exit tilt: 228.51 m / 35.15 s, burnout error ≈0.97°, 1° band at
  0.79 s, coast fin ≤8.9°;
* steady 5 m/s wind: 232.22 m, no fin saturation.

These are regression bounds on the coupled static-CFD/OpenRocket surrogate,
**not** flight qualification.

## Hardware-in-the-loop bench

`hil/` adds a local HIL bench that runs the **same C guidance core** as the
flight image (no Python avionics port). A Verilated peripheral model and C++
bridge consume shared-memory sensor frames from a physics plant that replays
the OpenRocket F32 flight and the recorded CFD lookup. If
`build/cfd/motion.json` exists from `make ci-live`, HIL uses it automatically;
otherwise it falls back to the committed `sim/data/cfd/fluidx3d` CSV fit. Override
with `rockets/s-lx/hil/run.sh --backend cfd --motion /path/to/motion.json`.

```sh
make -C rockets/s-lx/hil build   # Verilate RTL + build bridge (needs Verilator)
make -C rockets/s-lx/hil run     # full 100 Hz flight into hil/logs/cfd
make -C rockets/s-lx/hil vis     # browser GUI at http://127.0.0.1:8765
```

The GUI (`hil/vis/`) is a stdlib-only HTTP dashboard. Select run `cfd` to follow
`hil/logs/cfd`. It shows flight state, faults, sensor/trajectory estimates,
dynamics, state-transition events, the guidance output next to its delayed and
50 Hz latched versions, the actual fin angle, per-fin move-progress bars, PWM
pulse widths and a live pin/peripheral panel (I2C, SPI/SD CS, USART, ARM input,
status LED, four PCA9685 channels). Bridge and plant logs are tailed while a
run is in progress; the GUI never attaches to shared memory itself.

### Native CFD/HIL window (Rust, no crates)

`../../shared/vis/rust/` is a dependency-free Rust X11 client shared by every
rocket. It opens a **real Xwayland window** (raw X11 wire protocol over
`/tmp/.X11-unix/X0`, CPU framebuffer uploaded with `PutImage`). Labels, paths,
GPIO names and fin states come from `config/vis.json`; no airframe data is
compiled into the shared code. The layout is deliberately monochrome and
data-dense:

* **Live CFD** — a separate Taichi/CUDA process (`cfd-live`) streams macroscopic
  velocity slices at 5–10 Hz over a Unix socket. The visualiser draws RK2
  streamlines through the newest frame and displays its sequence number and
  rate. If the stream is unavailable it falls back to the recorded slices in
  `build/cfd/fields/`.
* **Live telemetry feed** — `hil/logs/bridge.csv` is tailed for time, state,
  altitude, vertical velocity, predicted apogee, attitude, fin commands, PWM
  widths and the real peripheral GPIO bits.
* **Rocket geometry** — the binary STL named in `config/vis.json`, with fins
  deflected about their radial hinges using the actual fin angles.

```sh
make -C rockets/s-lx cfd-fields          # regenerate recorded slices + motion.json
make -C rockets/s-lx cfd-live            # start the live Taichi field stream
make -C rockets/s-lx vis-rust            # cargo build of the shared visualiser
make -C rockets/s-lx vis-rust-run VIS_ARGS=--replay
make -C rockets/s-lx vis-rust-run VIS_ARGS='--hold --print-rows'
```

Useful binary flags: `--config PATH`, `--log PATH`, `--stl PATH`, `--fields DIR`,
`--motion PATH`, `--socket PATH`, `--fps N`, `--replay`, `--hold`,
`--print-rows`, and `--snapshot PPM [--snapshot-after SECS]`. The PPM is the
**exact framebuffer uploaded to X**. `--hold` ignores key presses; otherwise
any key in the window quits. `SLX_X_DEBUG=1` prints X11 request and error
diagnostics. The visualiser is a viewer only: it never writes telemetry or
shared memory.

Servo switching is **not** instantaneous in either simulator: each guidance
command waits 10 ms (one 100 Hz tick) before reaching the PCA9685 input, is
latched at the PCA9685's 50 Hz frame rate, slewed at 300 °/s and passed through
a 35 ms first-order lag. Each new latched target records its from/to angles and
a completion percentage, so the GUI shows a real move in progress rather than
a step change. The recorded CFD lookup is evaluated at **100 Hz** from those
actual fin angles; its force/drag/moment increments then pass through a 20 ms
reduced-order response lag so a steady-state coefficient never snaps instantly
to a new geometry. Running the Taichi LBM solver inside the 100 Hz loop would
not be physically meaningful at this fidelity (steady-state force coefficients,
no moving boundary) and would break the flight deadline; the GPU sweep remains
an offline calibration step feeding the transient plant.

`hil/wokwi/` adds two local-only Wokwi projects: a runnable STM32 smoke test
that links the unchanged guidance core (`make -C rockets/s-lx/hil wokwi-stm32`,
token kept outside the repo) and an ESP32-C3 wiring/lint project. Wokwi has no
BMP280/ICM20689/PCA9685 parts, so those transports are substituted; the control
law is not. See `hil/wokwi/README.md` for the exact substitution and serial
wiring notes.

CI/CD remains **local only**: `make -C rockets/s-lx ci`, `make -C rockets/s-lx
ci-live` and `rockets/s-lx/ci/run-local.sh` are the sole entry points. No GitHub
Actions workflow or PR gate exists.

Real ICM-20689/BMP280/PCA9685 parts, SD writes, servo timing, the 100 Hz
deadline, ejection and egg survival remain unverified. The ARM build was checked
with Newlib-nano/`nosys` as a substitute for upstream `picolibc.specs`;
upstream picolibc, flashing and hardware timing still need verification.

## Known open items

* Verify physical fin numbering: `xmix_p15` couples pitch and yaw.
* Canopy `Cd ≈ 0.865` / `CdA = 0.150` needs a ground drop/ejection test.
* Flutter check and ground-test the real ICM-20689/BMP280/PCA9685 + SD + servo
  timing on the flight PCB (the HIL substitutes these transports).
* True moving-boundary CFD is a separate SDK/production task (`cfd/README.md`).
