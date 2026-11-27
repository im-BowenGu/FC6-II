/*
 * UKROC 2027 STM32F103C8T6 "Blue Pill" flight target.
 *
 * Autonomous onboard guidance only: no radio, GNSS, pyro or ground-control
 * interface exists in this target. See rocket-profile/include/ukroc_profile.h
 * and rocket/AVIONICS.md §2.1 for the rule-13 posture.
 */

#ifndef _AO_PINS_H_
#define _AO_PINS_H_

/*
 * Blue Pill board: 8 MHz HSE crystal, PLL x9 -> 72 MHz SYSCLK.
 * USB is not used by the application; the flash loader still gets 48 MHz.
 */
#define AO_HSE                          1
#define AO_HSE_BYPASS                   0
#define AO_RCC_CFGR_USBPRE              STM_RCC_CFGR_USBPRE_1_5
#define AO_RCC_CFGR_PLLMUL              STM_RCC_CFGR_PLLMUL_9
#define AO_RCC_CFGR_PLLXTPRE            STM_RCC_CFGR_PLLXTPRE_1
#define AO_RCC_CFGR_PPRE2_DIV           STM_RCC_CFGR_PPRE2_DIV_1
#define AO_RCC_CFGR_PPRE1_DIV           STM_RCC_CFGR_PPRE1_DIV_2
#define AO_RCC_CFGR_HPRE_DIV            STM_RCC_CFGR_HPRE_DIV_1
#define AO_RCC_CFGR_ADCPRE              STM_RCC_CFGR_ADCPRE_6

#define AO_SYSCLK                       72000000UL
#define AO_HCLK                         72000000UL
#define AO_APB1CLK                      36000000UL
#define AO_APB2CLK                      72000000UL
#define AO_ADCCLK                       12000000UL

/* Kernel configuration. The guidance and logger each get a 768-byte stack. */
#define HAS_TASK                        1
#define HAS_TASK_INFO                   1
#define AO_NUM_TASKS                    4
#define AO_STACK_SIZE                   768
#define HAS_STACK_GUARD                 0
#define AO_CHECK_STACK                  0
#define HAS_SAMPLE_PROFILE              0
#define AO_PROFILE                      0

/* Rule-13 posture. */
#define HAS_RADIO                       0
#define HAS_TELEMETRY                   0
#define HAS_APRS                        0
#define HAS_COMPANION                   0
#define HAS_USB                         0
#define HAS_PYRO                        0
#define AO_PYRO_NUM                     0
#define HAS_GPS                         0
#define USE_EEPROM_CONFIG               0
#define USE_STORAGE_CONFIG              0
#define HAS_EEPROM                      0
#define HAS_LOG                         0

#define LOW_LEVEL_DEBUG                 0
#define HAS_BEEP                        0
#define PACKET_HAS_SLAVE                0
#define PACKET_HAS_MASTER               0

/* PC13 onboard LED is active low. The target-local ao_led.c handles that. */
#define HAS_LED                         1
#define LED_0_PORT                      (&stm_gpioc)
#define LED_0_PIN                       13
#define LED_0_ACTIVE_LOW                1
#define AO_LED_GREEN                    (1U << 0)
#define AO_LED_PANIC                    AO_LED_GREEN
#define LEDS_AVAILABLE                  AO_LED_GREEN

/* I2C1 on PB6/PB7: BMP280 0x76, ICM-20689 0x68, PCA9685 0x40. */
#define HAS_I2C_1                       1
#define I2C_1_PB6_PB7                   1
#define I2C_1_PB8_PB9                   0
#define HAS_I2C_2                       0
#define I2C_2_PB10_PB11                 0

/* SPI1 on PA5/PA6/PA7; microSD chip select is PA4, active low. */
#define HAS_SPI_1                       1
#define SPI_1_PA5_PA6_PA7               1
#define SPI_1_PB3_PB4_PB5               0
#define SPI_1_PE13_PE14_PE15            0
#define SPI_1_MODE_OUTPUT               STM_GPIO_CR_MODE_OUTPUT_10MHZ
#define HAS_SPI_2                       0
#define SPI_2_PB13_PB14_PB15            0
#define AO_SDCARD_SPI_BUS               (AO_SPI_1_PA5_PA6_PA7 | AO_SPI_MODE_0)
#define AO_SDCARD_SPI_CS_PORT           (&stm_gpioa)
#define AO_SDCARD_SPI_CS_PIN            4

/*
 * The shared SD-card driver still uses legacy AO_SPI_SPEED_* names and the
 * STM32F4-only stm_pupdr_set() helper. F103 selects SPI speeds through
 * ao_spi_speed() and has external wire pull-ups, so provide target-local
 * shims instead of changing shared firmware for every upstream target.
 * These macros are visible only while compiling this target.
 */
#define AO_SPI_SPEED_250kHz             ao_spi_speed(AO_SDCARD_SPI_BUS, 250000U)
#define AO_SPI_SPEED_FAST               ao_spi_speed(AO_SDCARD_SPI_BUS, 20000000U)
#define STM_PUPDR_PULL_UP               0
#define stm_pupdr_set(port, pin, mode)  do { (void) (port); (void) (pin); (void) (mode); } while (0)

/* PA1 battery divider (1:2) through ADC1 IN1. */
#define HAS_ADC                         0
#define HAS_ADC_SINGLE                  1
#define HAS_ADC_TEMP                    0
#define AO_NUM_ADC                      1
#define AO_ADC_SQ1                      1
#define AO_ADC_PIN0_PORT                (&stm_gpioa)
#define AO_ADC_PIN0_PIN                 1
#define AO_ADC_DUMP(p)                  ((void) (p))

struct ao_adc {
	int16_t				v_batt;
};

/* Wake the control task on each 100 Hz SysTick. */
void ukroc_timer_hook(void);
#define AO_TIMER_HOOK do {						\
		ukroc_timer_hook();					\
	} while (0)

/* Discrete UKROC pins, mirrored from ukroc_stm32_pinmap.h. */
#define AO_UKROC_ARM_PORT               (&stm_gpioa)
#define AO_UKROC_ARM_PIN                0
#define AO_UKROC_LED_PORT               LED_0_PORT
#define AO_UKROC_LED_PIN                LED_0_PIN

#endif /* _AO_PINS_H_ */
