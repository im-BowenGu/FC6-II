/*
 * Minimal ICM-20689 I2C IMU driver for the UKROC STM32 target.
 *
 * Full-scale ranges are chosen for a model rocket: ±16 g acceleration keeps
 * the F32 boost phase in range, and ±1000 °/s leaves gyro headroom. Changing
 * either range requires changing the matching scale below.
 */

#ifndef _AO_ICM20689_H_
#define _AO_ICM20689_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef AO_ICM20689_I2C_INDEX
#define AO_ICM20689_I2C_INDEX 0U
#endif

#ifndef AO_ICM20689_I2C_ADDR
#define AO_ICM20689_I2C_ADDR 0x68U
#endif

#define AO_ICM20689_ADDR_WRITE ((uint8_t) (AO_ICM20689_I2C_ADDR << 1))
#define AO_ICM20689_ADDR_READ  ((uint8_t) ((AO_ICM20689_I2C_ADDR << 1) | 1U))

/* ACCEL_FS_SEL = 3 (±16 g) and GYRO_FS_SEL = 2 (±1000 °/s). */
#define AO_ICM20689_ACCEL_CONFIG 0x18U
#define AO_ICM20689_ACCEL_SCALE  (1.0f / 2048.0f)
#define AO_ICM20689_GYRO_CONFIG  0x10U
#define AO_ICM20689_GYRO_SCALE   (1.0f / 32.8f)

struct ao_icm20689_sample {
	float ax_g;
	float ay_g;
	float az_g;
	float gx_dps;
	float gy_dps;
	float gz_dps;
};

bool
ao_icm20689_init(void);

bool
ao_icm20689_read(struct ao_icm20689_sample *sample);

#endif /* _AO_ICM20689_H_ */
