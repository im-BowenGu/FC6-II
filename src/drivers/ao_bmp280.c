/*
 * Copyright © 2026 UKROC STM32 integration
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include "ao.h"
#include "ao_bmp280.h"

struct ao_bmp280_cal {
	uint16_t	dig_t1;
	int16_t		dig_t2;
	int16_t		dig_t3;
	uint16_t	dig_p1;
	int16_t		dig_p2;
	int16_t		dig_p3;
	int16_t		dig_p4;
	int16_t		dig_p5;
	int16_t		dig_p6;
	int16_t		dig_p7;
	int16_t		dig_p8;
	int16_t		dig_p9;
};

static struct ao_bmp280_cal	ao_bmp280_calibration;
static int32_t			ao_bmp280_t_fine;

static uint16_t
ao_bmp280_le16(const uint8_t *p)
{
	return (uint16_t) (((uint16_t) p[1] << 8) | p[0]);
}

static int16_t
ao_bmp280_le16s(const uint8_t *p)
{
	return (int16_t) ao_bmp280_le16(p);
}

static bool
ao_bmp280_write(uint8_t reg, uint8_t value)
{
	uint8_t	d[2];
	uint8_t ok;

	d[0] = reg;
	d[1] = value;
	ao_i2c_get(AO_BMP280_I2C_INDEX);
	ok = ao_i2c_start(AO_BMP280_I2C_INDEX, AO_BMP280_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(d, 2, AO_BMP280_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_BMP280_I2C_INDEX);
	return ok != 0U;
}

static bool
ao_bmp280_read(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t ok;

	ao_i2c_get(AO_BMP280_I2C_INDEX);
	ok = ao_i2c_start(AO_BMP280_I2C_INDEX, AO_BMP280_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(&reg, 1, AO_BMP280_I2C_INDEX, 0);
	}
	if (ok) {
		ok = ao_i2c_start(AO_BMP280_I2C_INDEX, AO_BMP280_ADDR_READ);
	}
	if (ok) {
		ok = ao_i2c_recv(data, len, AO_BMP280_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_BMP280_I2C_INDEX);
	return ok != 0U;
}

bool
ao_bmp280_init(void)
{
	uint8_t id;
	uint8_t cal[24];
	struct ao_bmp280_cal *c = &ao_bmp280_calibration;

	if (!ao_bmp280_read(0xd0U, &id, 1U)) {
		return false;
	}
	/* 0x58 is BMP280; 0x60 is the register-compatible BME280. */
	if (id != 0x58U && id != 0x60U) {
		return false;
	}
	if (!ao_bmp280_read(0x88U, cal, sizeof(cal))) {
		return false;
	}

	c->dig_t1 = ao_bmp280_le16(&cal[0]);
	c->dig_t2 = ao_bmp280_le16s(&cal[2]);
	c->dig_t3 = ao_bmp280_le16s(&cal[4]);
	c->dig_p1 = ao_bmp280_le16(&cal[6]);
	c->dig_p2 = ao_bmp280_le16s(&cal[8]);
	c->dig_p3 = ao_bmp280_le16s(&cal[10]);
	c->dig_p4 = ao_bmp280_le16s(&cal[12]);
	c->dig_p5 = ao_bmp280_le16s(&cal[14]);
	c->dig_p6 = ao_bmp280_le16s(&cal[16]);
	c->dig_p7 = ao_bmp280_le16s(&cal[18]);
	c->dig_p8 = ao_bmp280_le16s(&cal[20]);
	c->dig_p9 = ao_bmp280_le16s(&cal[22]);

	ao_bmp280_t_fine = 0;
	/* Normal mode, x16 pressure and temperature oversampling. */
	return ao_bmp280_write(0xf4U, 0x27U) && ao_bmp280_write(0xf5U, 0x00U);
}

static int32_t
ao_bmp280_compensate_temperature(int32_t raw)
{
	struct ao_bmp280_cal *c = &ao_bmp280_calibration;
	int32_t var1;
	int32_t var2;
	int32_t t2;

	t2 = (raw >> 4) - (int32_t) c->dig_t1;
	var1 = ((((raw >> 3) - ((int32_t) c->dig_t1 << 1)) *
		(int32_t) c->dig_t2) >> 11);
	var2 = ((((t2 * t2) >> 12) * (int32_t) c->dig_t3) >> 14);
	ao_bmp280_t_fine = var1 + var2;
	return (ao_bmp280_t_fine * 5 + 128) >> 8;
}

static uint32_t
ao_bmp280_compensate_pressure(int32_t raw)
{
	struct ao_bmp280_cal *c = &ao_bmp280_calibration;
	int64_t var1;
	int64_t var2;
	int64_t p;

	var1 = (int64_t) ao_bmp280_t_fine - 128000;
	var2 = var1 * var1 * (int64_t) c->dig_p6;
	var2 += (var1 * (int64_t) c->dig_p5) << 17;
	var2 += ((int64_t) c->dig_p4) << 35;
	var1 = ((var1 * var1 * (int64_t) c->dig_p3) >> 8) +
		((var1 * (int64_t) c->dig_p2) << 12);
	var1 = (((int64_t) 1) << 47) + var1;
	if (var1 == 0) {
		return 0U;
	}

	p = 1048576 - (int64_t) raw;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) c->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) c->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t) c->dig_p7 << 4);
	if (p < 0) {
		p = 0;
	}
	return (uint32_t) p;
}

bool
ao_bmp280_read_pa(float *pressure_pa)
{
	uint8_t d[6];
	int32_t raw_p;
	int32_t raw_t;
	uint32_t pa;

	if (pressure_pa == NULL) {
		return false;
	}
	if (!ao_bmp280_read(0xf7U, d, sizeof(d))) {
		return false;
	}

	raw_p = (int32_t) (((uint32_t) d[0] << 12) |
			   ((uint32_t) d[1] << 4) |
			   ((uint32_t) d[2] >> 4));
	raw_t = (int32_t) (((uint32_t) d[3] << 12) |
			   ((uint32_t) d[4] << 4) |
			   ((uint32_t) d[5] >> 4));

	(void) ao_bmp280_compensate_temperature(raw_t);
	pa = ao_bmp280_compensate_pressure(raw_p);
	*pressure_pa = (float) pa;
	return *pressure_pa > 10000.0f && *pressure_pa < 200000.0f;
}
