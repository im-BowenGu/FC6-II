/*
 * S-LX STM32F103C8T6 autonomous flight application (UKROC 2027 competition).
 *
 * This is a dedicated flight target: there is no radio, GNSS, USB, command
 * console or pyro interface. The 100 Hz SysTick wakes one control task which
 * reads the commercial BMP280/ICM-20689 breakouts, runs the hardware-independent
 * guidance core from rocket-profile/, drives four PCA9685 servos and records a
 * CSV telemetry file on microSD.
 *
 * Rule 13 posture: see rocket/AVIONICS.md section 2.1. Parachute deployment is
 * left to the F32 motor's own ejection charge; this firmware commands no
 * pyrotechnic device and accepts no signal after lift-off.
 */

#include "ao.h"
#include "ao_adc_single.h"
#include "ao_bmp280.h"
#include "ao_icm20689.h"
#include "ao_pca9685.h"
#include "ao_fat.h"
#include "slx_profile.h"

#include <stdio.h>
#include <string.h>

/* 8.3 FAT name: "S-LX.CSV". */
#define AO_SLX_LOG_NAME	"S-LX   CSV"

/* ao_adc_single_get() inserts two 10 ms settling delays; do not call it at
 * 100 Hz. One sample per second is enough for a packet-voltage trend. */
#define AO_SLX_VBAT_SAMPLE_TICKS	100U

/* Sanity bound on the generated CSV row. */
#define AO_SLX_ROW_SIZE	256U

static struct ao_task		ao_slx_task;
static uint8_t			ao_slx_tick_wchan;
static slx_guidance_t		ao_slx_guidance;
static int8_t			ao_slx_log_fd = -1;
static float			ao_slx_vbat;

/*
 * No console exists on this target. These stubs satisfy the unconditional
 * command/stdio registration in shared STM32F1 drivers without linking the
 * interactive command shell or any serial/USB stdio backend.
 */
void
ao_cmd_register(const struct ao_cmds *cmds)
{
	(void) cmds;
}

int
ao_putchar(char c)
{
	(void) c;
	return 0;
}

char
ao_getchar(void)
{
	return 0;
}

void
flush(void)
{
}

uint8_t ao_stdin_ready;

/*
 * Diagnostic printf() calls in the shared STM32F1 drivers need a sink. The
 * flight image uses picolibc (as upstream does), where stdout is a FILE with
 * FDEV_SETUP_STREAM. newlib-nano is supported only for host-side verification
 * builds and uses the _write syscall instead. Neither path emits any bytes.
 */
#if defined(__PICOLIBC__)
static int
ao_slx_putc(char c, FILE *stream)
{
	(void) c;
	(void) stream;
	return 0;
}

static int
ao_slx_getc(FILE *stream)
{
	(void) stream;
	return 0;
}

static int
ao_slx_flushc(FILE *stream)
{
	(void) stream;
	return 0;
}

static FILE ao_slx_stdio = FDEV_SETUP_STREAM(ao_slx_putc,
					       ao_slx_getc,
					       ao_slx_flushc,
					       _FDEV_SETUP_RW);

FILE *const stdin = &ao_slx_stdio;
FILE *const stdout = &ao_slx_stdio;
FILE *const stderr = &ao_slx_stdio;
#elif defined(__NEWLIB__)
int _write(int fd, char *buf, int len);

int
_write(int fd, char *buf, int len)
{
	(void) fd;
	(void) buf;
	return len;
}
#endif

/* Called from the 100 Hz SysTick ISR. ao_wakeup() is ISR-safe. */
void
slx_timer_hook(void)
{
	ao_wakeup(&ao_slx_tick_wchan);
}

static void
ao_slx_write_fins(const float fins[4])
{
	uint8_t channel;

	for (channel = 0U; channel < AO_PCA9685_NUM_CHANNELS; channel++) {
		float us = SLX_SERVO_CENTER_US + fins[channel] * SLX_SERVO_US_PER_DEG;

		(void) ao_pca9685_set_us(channel, us);
	}
}

static void
ao_slx_log_close(void)
{
	if (ao_slx_log_fd >= 0) {
		(void) ao_fat_sync();
		(void) ao_fat_close(ao_slx_log_fd);
		ao_slx_log_fd = -1;
	}
}

static void
ao_slx_log_row(const slx_snapshot_t *snap)
{
	char row[AO_SLX_ROW_SIZE];
	size_t len;

	if (ao_slx_log_fd < 0) {
		return;
	}
	len = slx_format_csv_row(row, sizeof(row), snap, ao_slx_vbat);
	if (len != 0U) {
		(void) ao_fat_write(ao_slx_log_fd, row, (int) len);
	}
}

static void
ao_slx_control(void)
{
	uint16_t adc_ticks = 0U;
	float pressure_pa = 101325.0f;
	struct ao_icm20689_sample raw;
	slx_imu_t imu;
	slx_snapshot_t snap;
	bool arm_closed;

	for (;;) {
		ao_sleep(&ao_slx_tick_wchan);

		memset(&raw, 0, sizeof(raw));
		if (!ao_bmp280_read_pa(&pressure_pa) || !ao_icm20689_read(&raw)) {
			/*
			 * Do not advance the guidance state on bad sensor data.
			 * After ten consecutive failures the core forces FAULT and
			 * centres the fins; keep writing that neutral command.
			 */
			(void) slx_guidance_note_sensor_failure(&ao_slx_guidance);
			ao_slx_write_fins(ao_slx_guidance.fins_deg);
			if (ao_slx_guidance.state == SLX_STATE_FAULT) {
				ao_slx_log_close();
			}
			continue;
		}

		imu.ax_g = raw.ax_g;
		imu.ay_g = raw.ay_g;
		imu.az_g = raw.az_g;
		imu.gx_dps = raw.gx_dps;
		imu.gy_dps = raw.gy_dps;
		imu.gz_dps = raw.gz_dps;

		/* PA0 arm switch is active low with the internal pull-up. */
		arm_closed = ao_gpio_get(AO_SLX_ARM_PORT, AO_SLX_ARM_PIN) == 0U;

		/* ao_time() is in 10 ms ticks; guidance works in milliseconds. */
		snap = slx_guidance_tick(&ao_slx_guidance,
					   (uint32_t) ao_time() * 10U,
					   arm_closed, pressure_pa, imu);

		ao_slx_write_fins(snap.fins_deg);

		if (++adc_ticks >= AO_SLX_VBAT_SAMPLE_TICKS) {
			struct ao_adc adc;

			adc_ticks = 0U;
			ao_adc_single_get(&adc);
			/* 12-bit sample, 3.3 V reference, 1:2 divider. */
			ao_slx_vbat = (float) adc.v_batt * 3.3f * 2.0f / 4096.0f;
		}

		ao_slx_log_row(&snap);
		if (snap.close_log) {
			ao_slx_log_close();
			ao_led_on(AO_LED_GREEN);
		}
	}
}

static bool
ao_slx_log_open(void)
{
	char name[11] = { 'U', 'K', 'R', 'O', 'C', ' ', ' ', ' ', 'C', 'S', 'V' };
	int8_t fd;

	fd = ao_fat_creat(name);
	if (fd < 0) {
		return false;
	}
	ao_slx_log_fd = fd;
	(void) ao_fat_write(fd, (void *) SLX_CSV_HEADER "\r\n",
			    (int) (sizeof(SLX_CSV_HEADER) + 1U));
	return true;
}

int
main(void)
{
	bool fault = false;
	float pressure_pa;

	ao_clock_init();
	ao_task_init();
	ao_led_init();
	ao_led_on(LEDS_AVAILABLE);
	ao_timer_init();

	ao_i2c_init();
	ao_spi_init();
	ao_dma_init();

	ao_enable_input(AO_SLX_ARM_PORT, AO_SLX_ARM_PIN, AO_EXTI_MODE_PULL_UP);
	ao_adc_single_init();

	/* FAT init also initialises the microSD card through ao_bufio_init(). */
	ao_fat_init();
	if (!ao_slx_log_open()) {
		slx_guidance_force_fault(&ao_slx_guidance, SLX_FAULT_LOG_INIT);
		fault = true;
	}

	/* Centre all four servos before the arm switch is closed. */
	if (!ao_pca9685_init() || !ao_pca9685_all_us(SLX_SERVO_CENTER_US)) {
		slx_guidance_force_fault(&ao_slx_guidance, SLX_FAULT_PCA9685_INIT);
		fault = true;
	}
	if (!ao_bmp280_init()) {
		slx_guidance_force_fault(&ao_slx_guidance, SLX_FAULT_BMP280_INIT);
		fault = true;
	}
	if (!ao_icm20689_init()) {
		slx_guidance_force_fault(&ao_slx_guidance, SLX_FAULT_ICM20689_INIT);
		fault = true;
	}

	if (!fault) {
		slx_guidance_init(&ao_slx_guidance);
		/* Pad pressure is the sea-level reference for this flight. */
		if (ao_bmp280_read_pa(&pressure_pa)) {
			slx_guidance_set_sea_level_pressure(&ao_slx_guidance,
							      pressure_pa);
		}
		slx_guidance_enter_armed(&ao_slx_guidance);
	}

	ao_led_off(LEDS_AVAILABLE);
	ao_add_task(&ao_slx_task, ao_slx_control, "slx");
	ao_start_scheduler();
	return 0;
}
