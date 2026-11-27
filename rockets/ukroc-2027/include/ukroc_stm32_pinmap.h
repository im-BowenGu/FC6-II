/*
 * UKROC 2027 STM32F103C8T6 "Blue Pill" pin map.
 *
 * These are STM32duino-style port/bit names, not AltOS ao_pins.h macros. They
 * reproduce the working flight-stack reference in
 * rocket/firmware/stm32f103_guidance/stm32f103_guidance.ino.
 */

#ifndef UKROC_STM32_PINMAP_H
#define UKROC_STM32_PINMAP_H

/* GPIO ports as numeric identifiers for generated configuration tables. */
#define UKROC_STM32_PORT_A   0U
#define UKROC_STM32_PORT_B   1U
#define UKROC_STM32_PORT_C   2U

/* I2C1: sensors and servo driver. */
#define UKROC_STM32_I2C_SDA_PORT   UKROC_STM32_PORT_B
#define UKROC_STM32_I2C_SDA_PIN    7U        /* PB7 */
#define UKROC_STM32_I2C_SCL_PORT   UKROC_STM32_PORT_B
#define UKROC_STM32_I2C_SCL_PIN    6U        /* PB6 */
#define UKROC_STM32_I2C_HZ         400000UL

#define UKROC_STM32_BMP280_ADDR    0x76U
#define UKROC_STM32_ICM20689_ADDR  0x68U
#define UKROC_STM32_PCA9685_ADDR   0x40U
#define UKROC_STM32_SERVO_CHANNELS 4U

/* SPI1: microSD. */
#define UKROC_STM32_SPI_SCK_PORT   UKROC_STM32_PORT_A
#define UKROC_STM32_SPI_SCK_PIN    5U        /* PA5 */
#define UKROC_STM32_SPI_MISO_PORT  UKROC_STM32_PORT_A
#define UKROC_STM32_SPI_MISO_PIN   6U        /* PA6 */
#define UKROC_STM32_SPI_MOSI_PORT  UKROC_STM32_PORT_A
#define UKROC_STM32_SPI_MOSI_PIN   7U        /* PA7 */
#define UKROC_STM32_SD_CS_PORT     UKROC_STM32_PORT_A
#define UKROC_STM32_SD_CS_PIN      4U        /* PA4, active low */

/* Discrete inputs/outputs. */
#define UKROC_STM32_ARM_PORT       UKROC_STM32_PORT_A
#define UKROC_STM32_ARM_PIN        0U        /* PA0, input pull-up */
#define UKROC_STM32_ARM_ACTIVE_LOW 1U
#define UKROC_STM32_LED_PORT       UKROC_STM32_PORT_C
#define UKROC_STM32_LED_PIN        13U       /* PC13, onboard LED, active low */
#define UKROC_STM32_LED_ACTIVE_LOW 1U
#define UKROC_STM32_VBAT_PORT      UKROC_STM32_PORT_A
#define UKROC_STM32_VBAT_PIN       1U        /* PA1, ADC1, 1:2 divider */
#define UKROC_STM32_ADC_MAX        4095U
#define UKROC_STM32_VREF_MV        3300U
#define UKROC_STM32_VBAT_DIVISOR   2U

/* Programming-only pins must be disconnected before flight. */
#define UKROC_STM32_SWDIO_PORT     UKROC_STM32_PORT_A
#define UKROC_STM32_SWDIO_PIN      13U
#define UKROC_STM32_SWCLK_PORT     UKROC_STM32_PORT_A
#define UKROC_STM32_SWCLK_PIN      14U

/* No radio, GNSS or pyro pins exist in this profile. */
#define UKROC_STM32_RADIO_PINS     0U
#define UKROC_STM32_GPS_PINS       0U
#define UKROC_STM32_PYRO_PINS      0U

#endif /* UKROC_STM32_PINMAP_H */
