/*
 * Copyright © 2026 UKROC STM32 integration
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include "ao.h"
#include "ao_pca9685.h"

#define AO_PCA9685_MODE1        0x00U
#define AO_PCA9685_PRESCALE     0xfeU
#define AO_PCA9685_LED0_ON_L    0x06U

#define AO_PCA9685_MODE1_SLEEP  0x10U
#define AO_PCA9685_MODE1_AI     0x01U
#define AO_PCA9685_PRESCALE_50HZ 121U

static bool
ao_pca9685_write(uint8_t reg, uint8_t value)
{
	uint8_t d[2];
	uint8_t ok;

	d[0] = reg;
	d[1] = value;
	ao_i2c_get(AO_PCA9685_I2C_INDEX);
	ok = ao_i2c_start(AO_PCA9685_I2C_INDEX, AO_PCA9685_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(d, 2, AO_PCA9685_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_PCA9685_I2C_INDEX);
	return ok != 0U;
}

bool
ao_pca9685_init(void)
{
	/*
	 * Put the part to sleep before changing prescale. All LED registers
	 * start at zero duty; the caller must write 1500 µs before enabling
	 * outputs. The OE pin is owned by the airframe harness.
	 */
	if (!ao_pca9685_write(AO_PCA9685_MODE1, AO_PCA9685_MODE1_SLEEP)) {
		return false;
	}
	ao_delay(AO_MS_TO_TICKS(1U));
	if (!ao_pca9685_write(AO_PCA9685_PRESCALE, AO_PCA9685_PRESCALE_50HZ)) {
		return false;
	}
	if (!ao_pca9685_write(AO_PCA9685_MODE1, 0x00U)) {
		return false;
	}
	ao_delay(AO_MS_TO_TICKS(1U));
	return ao_pca9685_write(AO_PCA9685_MODE1, AO_PCA9685_MODE1_AI);
}

bool
ao_pca9685_set_us(uint8_t channel, float us)
{
	uint8_t d[5];
	uint16_t ticks;
	uint8_t ok;

	if (channel >= AO_PCA9685_NUM_CHANNELS) {
		return false;
	}
	if (us < 900.0f) {
		us = 900.0f;
	}
	if (us > 2100.0f) {
		us = 2100.0f;
	}
	ticks = (uint16_t) (us * 4096.0f / 20000.0f);

	d[0] = (uint8_t) (AO_PCA9685_LED0_ON_L + 4U * channel);
	d[1] = 0x00U;
	d[2] = 0x00U;
	d[3] = (uint8_t) (ticks & 0xffU);
	d[4] = (uint8_t) (ticks >> 8);

	ao_i2c_get(AO_PCA9685_I2C_INDEX);
	ok = ao_i2c_start(AO_PCA9685_I2C_INDEX, AO_PCA9685_ADDR_WRITE);
	if (ok) {
		ok = ao_i2c_send(d, sizeof(d), AO_PCA9685_I2C_INDEX, 1);
	}
	ao_i2c_put(AO_PCA9685_I2C_INDEX);
	return ok != 0U;
}

bool
ao_pca9685_all_us(float us)
{
	uint8_t channel;

	for (channel = 0U; channel < AO_PCA9685_NUM_CHANNELS; channel++) {
		if (!ao_pca9685_set_us(channel, us)) {
			return false;
		}
	}
	return true;
}
