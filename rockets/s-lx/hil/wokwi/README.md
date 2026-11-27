# S-LX Wokwi projects

Two local-only Wokwi projects complement the authoritative Verilator + shared
memory HIL in `..` (`make -C .. run`). They require the Wokwi CLI and a personal
token; **no token is stored in this repository**.

| Project | Purpose | Run |
| --- | --- | --- |
| `stm32f103/` | Runnable smoke test that links the **unchanged real C guidance core** (`rockets/s-lx/src/slx_profile.c` + `slx_aero.c`) and emits the real CSV telemetry format | `make -C .. wokwi-stm32` |
| `esp32-c3/` | Board/wiring and scenario lint for the unchanged Rust flight crate | `make -C .. wokwi-lint` |

## What is real and what is substituted

Wokwi does not provide BMP280, ICM20689 or PCA9685 parts. The STM32 harness
therefore substitutes:

* **BMP180** (`board-bmp180`) for the BMP280 barometer,
* **MPU6050** (`wokwi-mpu6050`) for the ICM20689 IMU,
* scenario-driven values for the I2C sensor readings, and
* direct GPIO servo commands instead of a PCA9685 driver.

Only the sensor/servo *transport* is substituted. The flight law, state machine,
fin mixing, apogee prediction and CSV row formatting are compiled directly from
the same sources used by the flight image and the Verilator HIL. The harness is
therefore a smoke test of the real controller, **not** a second Python avionics
implementation.

The ESP32-C3 project is intentionally **lint/wiring-only**: the unchanged Rust
firmware needs the `riscv32imc-unknown-none-elf` Rust target (not installed in
the local toolchain), and the flight crate expects a BMP280 at `0x76` while the
Wokwi BMP180 sits at `0x77`. A Wokwi-only sensor shim would be needed before an
ESP32-C3 ELF could fly the guidance logic; the Verilator HIL remains the
real-code reference.

## Serial monitor wiring

The `board-stm32-bluepill` part has no `RX`/`TX` pin aliases, so Wokwi CLI's
automatic `mcu:TX`/`mcu:RX` wiring targets nonexistent pins and no serial output
is captured. The STM32 diagram wires USART1 explicitly:

* `mcu:A9` (USART1_TX) -> `$serialMonitor:RX`
* `mcu:A10` (USART1_RX) -> `$serialMonitor:TX`

This is why the harness prints its boot banner and CSV rows on USART1.

## Running

```sh
# Lint both projects (no token needed)
make -C rockets/s-lx/hil wokwi-lint

# Run the STM32 real-guidance smoke test (token read from outside the repo)
make -C rockets/s-lx/hil wokwi-stm32
```

Override `WOKWI_CLI` and `WOKWI_TOKEN_ENV` if the CLI or token live elsewhere:

```sh
make -C rockets/s-lx/hil wokwi-stm32 \
  WOKWI_CLI=/path/to/wokwi-cli \
  WOKWI_TOKEN_ENV=/path/to/token.env
```

Lint reports only informational "undocumented type" notices for `board-bmp180`
and `board-esp32-c3-devkitm-1`; these are Wokwi's substitute parts and are
expected.
