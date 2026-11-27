/*
 * Copyright © 2026 UKROC STM32 integration
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include "ao.h"
#include "ao_icm20689.h"

static bool
ao_icm20689_write(uint8_t reg, uint8_t value)
{
	uint8_t d[2];
	uint8_t ok;

	d[0] = reg;
	d[1] = value;
	ao_i2c_get(AO_ICM20689_I2C_INDEX);
	ok = ao_i2c_start(AO_ICM20689_I2C_INDEX, AO_ICM20689_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(d, 2, AO_ICM20689_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_ICM20689_I2C_INDEX);
	return ok != 0U;
}

static bool
ao_icm20689_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t ok;

	ao_i2c_get(AO_ICM20689_I2C_INDEX);
	ok = ao_i2c_start(AO_ICM20689_I2C_INDEX, AO_ICM20689_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(&reg, 1, AO_ICM20689_I2C_INDEX, 0);
	}
	if (ok) {
		ok = ao_i2c_start(AO_ICM20689_I2C_INDEX, AO_ICM20689_ADDR_READ);
	}
	if (ok) {
		ok = ao_i2c_recv(data, len, AO_ICM20689_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_ICM20689_I2C_INDEX);
	return ok != 0U;
}

static int16_t
ao_icm20689_le16(const uint8_t *p)
{
	return (int16_t) (((uint16_t) p[0] << 8) | p[1]);
}

bool
ao_icm20689_init(void)
{
	uint8_t who_am_i;

	if (!ao_icm20689_read_regs(0x75U, &who_am_i, 1U)) {
		return false;
	}
	if (who_am_i != 0x38U) {
		return false;
	}

	if (!ao_icm20689_write(0x6bU, 0x80U)) {
		return false;
	}
	ao_delay(AO_MS_TO_TICKS(60U));

	return ao_icm20689_write(0x6bU, 0x01U) &&
	       ao_icm20689_write(0x1aU, 0x03U) &&
	       ao_icm20689_write(0x1bU, AO_ICM20689_GYRO_CONFIG) &&
	       ao_icm20689_write(0x1cU, AO_ICM20689_ACCEL_CONFIG);
}

bool
ao_icm20689_read(struct ao_icm20689_sample *sample)
{
	uint8_t d[14];

	if (sample == NULL) {
		return false;
	}
	if (!ao_icm20689_read_regs(0x3bU, d, sizeof(d))) {
		return false;
	}

	sample->ax_g = (float) ao_icm20689_le16(&d[0]) * AO_ICM20689_ACCEL_SCALE;
	sample->ay_g = (float) ao_icm20689_le16(&d[2]) * AO_ICM20689_ACCEL_SCALE;
	sample->az_g = (float) ao_icm20689_le16(&d[4]) * AO_ICM20689_ACCEL_SCALE;
	sample->gx_dps = (float) ao_icm20689_le16(&d[8]) * AO_ICM20689_GYRO_SCALE;
	sample->gy_dps = (float) ao_icm20689_le16(&d[10]) * AO_ICM20689_GYRO_SCALE;
	sample->gz_dps = (float) ao_icm20689_le16(&d[12]) * AO_ICM20689_GYRO_SCALE;
	return true;
}
