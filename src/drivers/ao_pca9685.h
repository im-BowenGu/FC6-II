/*
 * Minimal PCA9685 16-channel PWM driver for the S-LX STM32 target.
 *
 * The four aft fins are channels 0..3. Commands are pulse widths in
 * microseconds; 1500 µs is mechanical centre.
 */

#ifndef _AO_PCA9685_H_
#define _AO_PCA9685_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef AO_PCA9685_I2C_INDEX
#define AO_PCA9685_I2C_INDEX 0U
#endif

#ifndef AO_PCA9685_I2C_ADDR
#define AO_PCA9685_I2C_ADDR 0x40U
#endif

#define AO_PCA9685_ADDR_WRITE ((uint8_t) (AO_PCA9685_I2C_ADDR << 1))
#define AO_PCA9685_ADDR_READ  ((uint8_t) ((AO_PCA9685_I2C_ADDR << 1) | 1U))

#define AO_PCA9685_NUM_CHANNELS 4U

bool
ao_pca9685_init(void);

bool
ao_pca9685_set_us(uint8_t channel, float us);

bool
ao_pca9685_all_us(float us);

#endif /* _AO_PCA9685_H_ */
