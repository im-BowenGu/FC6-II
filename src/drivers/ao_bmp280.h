/*
 * Minimal BMP280 I2C barometer driver for the S-LX STM32 target.
 *
 * The device is a commercially available breakout on I2C1 at 0x76. The driver
 * uses the AltOS STM32F1 I2C API directly; it has no radio, GPS or pyro side
 * effects.
 */

#ifndef _AO_BMP280_H_
#define _AO_BMP280_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef AO_BMP280_I2C_INDEX
#define AO_BMP280_I2C_INDEX 0U
#endif

#ifndef AO_BMP280_I2C_ADDR
#define AO_BMP280_I2C_ADDR 0x76U
#endif

#define AO_BMP280_ADDR_WRITE ((uint8_t) (AO_BMP280_I2C_ADDR << 1))
#define AO_BMP280_ADDR_READ  ((uint8_t) ((AO_BMP280_I2C_ADDR << 1) | 1U))

bool
ao_bmp280_init(void);

bool
ao_bmp280_read_pa(float *pressure_pa);

#endif /* _AO_BMP280_H_ */
