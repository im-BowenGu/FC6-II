/*
 * S-LX HIL bridge: Verilated peripheral RTL + the real hardware-independent
 * STM32 guidance core (rockets/s-lx/src/slx_profile.c), connected to the
 * physics plant through POSIX shared memory. No Python re-implementation of
 * the avionics is involved in this path.
 */
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "verilated.h"
#include "Vslx_periph.h"

extern "C" {
#include "slx_profile.h"
#include "slx_hil_shm.h"
}

static const char *SENSOR_NAME = SLX_HIL_SENSORS_NAME;
static const char *FINS_NAME = SLX_HIL_FINS_NAME;

/* The firmware CSV schema stays unchanged. The HIL bridge appends pin/peripheral
 * telemetry after it so the GUI can show which outputs are actually active. */
static const char *HIL_EXTRA_CSV_HEADER =
    ",pwm_en,pwm0,pwm1,pwm2,pwm3,pwm0_us,pwm1_us,pwm2_us,pwm3_us,"
    "i2c_busy,spi_busy,sd_cs_n,usart_tx,usart_rx,status_led,"
    "arm_gpio,arm_led,fault_led,fault_code,sensor_faults";

static void *map_shm(const char *name, size_t size)
{
    int fd = shm_open(name, O_RDWR | O_CREAT, 0600);
    if (fd < 0) { perror("shm_open"); exit(1); }
    if (ftruncate(fd, (off_t)size) != 0) { perror("ftruncate"); exit(1); }
    void *p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) { perror("mmap"); exit(1); }
    close(fd);
    return p;
}

static bool read_sensors(slx_hil_sensors_t *shm, slx_hil_sensors_t *out)
{
    uint64_t a, b;
    do {
        a = shm->seq;
        if (a & 1U) continue;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        *out = *shm;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        b = shm->seq;
    } while (a != b);
    return true;
}

static void write_fins(slx_hil_fins_t *shm, double t_s,
                       const float cmd[4], const float act[4], slx_state_t state)
{
    uint64_t seq = __atomic_load_n(&shm->seq, __ATOMIC_ACQUIRE);
    __atomic_store_n(&shm->seq, seq + 1, __ATOMIC_RELEASE);
    shm->t_s = t_s;
    for (int i = 0; i < 4; i++) {
        shm->fin_cmd_deg[i] = cmd[i];
        shm->fin_act_deg[i] = act[i];
    }
    shm->state = (uint32_t)state;
    shm->flags = SLX_HIL_FLAG_RUNNING |
                 (state == SLX_STATE_FAULT ? SLX_HIL_FLAG_FAULT : 0U);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&shm->seq, seq + 2, __ATOMIC_RELEASE);
}

static void reg_write(Vslx_periph *d, uint8_t addr, uint64_t bits)
{
    d->wr = 1; d->valid = 1; d->addr = addr; d->wdata = (unsigned long)bits;
    d->clk = 0; d->eval();
    d->clk = 1; d->eval();
    d->wr = 0; d->valid = 0; d->eval();
}

static uint64_t as_bits(double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void write_reg_double(Vslx_periph *d, uint8_t addr, double value)
{
    reg_write(d, addr, as_bits(value));
}

static uint64_t arg_u64(int argc, char **argv, const char *key, uint64_t def)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], key) == 0)
            return strtoull(argv[i + 1], nullptr, 10);
    }
    return def;
}

static const char *arg_str(int argc, char **argv, const char *key, const char *def)
{
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], key) == 0)
            return argv[i + 1];
    }
    return def;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    slx_hil_sensors_t *sensors = (slx_hil_sensors_t *)map_shm(
        SENSOR_NAME, sizeof(slx_hil_sensors_t));
    slx_hil_fins_t *fins = (slx_hil_fins_t *)map_shm(
        FINS_NAME, sizeof(slx_hil_fins_t));

    Vslx_periph *dut = new Vslx_periph;
    dut->rst = 1; dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->rst = 0; dut->clk = 0; dut->eval();
    // Raw control bits: PWM enable, SPI/SD logging, I2C activity, USART TX.
    // Writing a double here would set the IEEE-754 mantissa instead of bit0.
    reg_write(dut, 0x00, 0x0fU);

    slx_guidance_t g;
    slx_guidance_init(&g);

    uint64_t max_ms = arg_u64(argc, argv, "--max-ms", 45000);
    const char *log_path = arg_str(argc, argv, "--out", "hil/logs/bridge.csv");
    FILE *log = fopen(log_path, "w");
    if (!log) { perror("fopen"); return 1; }
    setvbuf(log, nullptr, _IONBF, 0); // the GUI tails this file while running
    fprintf(log, "%s%s\n", SLX_CSV_HEADER, HIL_EXTRA_CSV_HEADER);

    bool sea_level_set = false;
    bool armed = false;
    bool processed_any = false;
    uint64_t ticks = 0;
    uint64_t last_t_ms = 0;
    uint64_t waits = 0;
    uint64_t dups = 0;
    double servo_us[4] = { SLX_SERVO_CENTER_US, SLX_SERVO_CENTER_US,
                           SLX_SERVO_CENTER_US, SLX_SERVO_CENTER_US };

    for (;;) {
        slx_hil_sensors_t s;
        read_sensors(sensors, &s);
        if (s.flags & SLX_HIL_FLAG_DONE) break;
        /* The plant creates and owns the sensor timeline. Before its first
         * published frame the segment is zero-filled; consuming that frame
         * would pin now_ms at zero forever. */
        if (!(s.flags & SLX_HIL_FLAG_RUNNING)) {
            if (++waits > 10000U) {
                fprintf(stderr, "bridge: timed out waiting for plant\n");
                break;
            }
            struct timespec wait = {0, 1000000};
            nanosleep(&wait, nullptr);
            continue;
        }
        uint64_t now_ms = (uint64_t)(s.t_s * 1000.0);
        if (now_ms > max_ms) break;
        if (processed_any && now_ms <= last_t_ms) {
            dups++;
            struct timespec wait = {0, 1000000};
            nanosleep(&wait, nullptr);
            continue;
        }
        last_t_ms = now_ms;
        processed_any = true;

        if (!sea_level_set) {
            slx_guidance_set_sea_level_pressure(&g, (float)s.pressure_pa);
            slx_guidance_enter_armed(&g);
            sea_level_set = true;
        }
        if (!armed) {
            // ARM must be closed before launch detection; the plant models it
            // as permanently closed for the duration of a guided flight.
            armed = true;
        }

        // Load emulated sensor inputs into the Verilated peripheral board.
        write_reg_double(dut, 0x10, s.alt_m);
        write_reg_double(dut, 0x11, s.pressure_pa);
        write_reg_double(dut, 0x12, s.vx_mps);
        write_reg_double(dut, 0x13, s.vy_mps);
        write_reg_double(dut, 0x14, s.vz_mps);
        write_reg_double(dut, 0x15, s.pitch_deg);
        write_reg_double(dut, 0x16, s.roll_deg);
        write_reg_double(dut, 0x17, s.yaw_deg);
        write_reg_double(dut, 0x18, s.p_rate_dps);
        write_reg_double(dut, 0x19, s.r_rate_dps);
        write_reg_double(dut, 0x1a, s.y_rate_dps);
        write_reg_double(dut, 0x1b, s.ax_g);
        write_reg_double(dut, 0x1c, s.ay_g);
        write_reg_double(dut, 0x1d, s.az_g);
        write_reg_double(dut, 0x1e, s.gx_dps);
        write_reg_double(dut, 0x1f, s.gy_dps);
        write_reg_double(dut, 0x20, s.gz_dps);
        write_reg_double(dut, 0x21, s.q_ratio);

        slx_imu_t imu = {};
        imu.ax_g = (float)s.ax_g; imu.ay_g = (float)s.ay_g; imu.az_g = (float)s.az_g;
        imu.gx_dps = (float)s.gx_dps; imu.gy_dps = (float)s.gy_dps; imu.gz_dps = (float)s.gz_dps;
        slx_snapshot_t snap = slx_guidance_tick(&g, (uint32_t)now_ms, true,
                                                (float)s.pressure_pa, imu);

        for (int i = 0; i < 4; i++) {
            double us = SLX_SERVO_CENTER_US +
                        snap.fins_deg[i] * (600.0 / SLX_MAX_FIN_DEG);
            if (us < SLX_SERVO_MIN_US) us = SLX_SERVO_MIN_US;
            if (us > SLX_SERVO_MAX_US) us = SLX_SERVO_MAX_US;
            servo_us[i] = us;
            // pwm_cmp[] is a raw 16-bit microsecond count, not an IEEE-754 word.
            reg_write(dut, (uint8_t)(0x30 + i), (uint64_t)us);
        }
        // Discrete pin states exposed to the GUI: arm input and fault latch.
        reg_write(dut, 0x38, armed ? 1U : 0U);
        reg_write(dut, 0x39, (uint64_t)g.fault);
        dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval();

        write_fins(fins, s.t_s, snap.fins_deg, snap.fins_deg, snap.state);

        char row[640];
        size_t row_len = slx_format_csv_row(row, sizeof(row), &snap, 3.9f);
        if (row_len >= 2U && row[row_len - 2U] == '\r' &&
                row[row_len - 1U] == '\n') {
            row[row_len - 2U] = '\0';  /* strip CRLF; append HIL telemetry */
        }
        fprintf(log,
                "%s,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,"
                "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                row,
                (unsigned)(dut->pwm_en ? 1 : 0),
                (unsigned)((dut->pwm >> 0) & 1U),
                (unsigned)((dut->pwm >> 1) & 1U),
                (unsigned)((dut->pwm >> 2) & 1U),
                (unsigned)((dut->pwm >> 3) & 1U),
                servo_us[0], servo_us[1], servo_us[2], servo_us[3],
                (unsigned)(dut->i2c_busy ? 1 : 0),
                (unsigned)(dut->spi_busy ? 1 : 0),
                (unsigned)(dut->sd_cs_n ? 1 : 0),
                (unsigned)(dut->usart_tx ? 1 : 0),
                (unsigned)(dut->usart_rx ? 1 : 0),
                (unsigned)(dut->status_led ? 1 : 0),
                (unsigned)(dut->arm_gpio ? 1 : 0),
                (unsigned)(dut->arm_led ? 1 : 0),
                (unsigned)(dut->fault_led ? 1 : 0),
                (unsigned)g.fault,
                (unsigned)g.sensor_faults);
        ticks++;

        if (snap.state == SLX_STATE_LANDED || snap.state == SLX_STATE_FAULT)
            break;

        /* Do not pace the bridge to wall-clock 100 Hz: the plant already
         * publishes 100 Hz simulation time and both processes share a CPU.
         * Sleeping here makes the bridge fall behind the plant and miss the
         * tail of the flight (the plant waits for this echo before DONE). */
    }

    slx_hil_fins_t done_copy = *fins;
    uint64_t seq = __atomic_load_n(&fins->seq, __ATOMIC_ACQUIRE);
    __atomic_store_n(&fins->seq, seq + 1, __ATOMIC_RELEASE);
    done_copy.flags |= SLX_HIL_FLAG_DONE;
    *fins = done_copy;
    __atomic_store_n(&fins->seq, seq + 2, __ATOMIC_RELEASE);

    fprintf(stderr, "bridge: %llu ticks, %llu duplicate waits, %llu startup waits, "
            "final state %s\n", (unsigned long long)ticks,
            (unsigned long long)dups, (unsigned long long)waits,
            slx_state_name(g.state));
    fclose(log);
    delete dut;
    return 0;
}
