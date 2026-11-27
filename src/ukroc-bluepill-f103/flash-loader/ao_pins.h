/*
 * Flash-loader pin configuration for the UKROC Blue Pill target.
 *
 * The board has an 8 MHz HSE crystal (no PLL pre-divider) and no USB pull-up
 * transistor on the standard Blue Pill, so HAS_USB_PULLUP is disabled. Flashing
 * requires a board variant with a 1.5k USB pull-up or an SWD programmer.
 *
 * Holding the PA0 arm switch closed at power-on keeps the loader active; with
 * the switch open PA0 reads high via its pull-up and the loader chains to the
 * application.
 */

#ifndef _AO_PINS_H_
#define _AO_PINS_H_

#define AO_HSE			1
#define AO_HSE_BYPASS		0

#define AO_SYSCLK		72000000
#define AO_HCLK			72000000
#define AO_APB1CLK		36000000
#define AO_APB2CLK		72000000
#define AO_ADCCLK		12000000

/* 8 MHz HSE x 9 / 1 = 72 MHz SYSCLK; USB = 72 / 1.5 = 48 MHz. */
#define AO_RCC_CFGR_USBPRE	STM_RCC_CFGR_USBPRE_1_5
#define AO_RCC_CFGR_PLLMUL	STM_RCC_CFGR_PLLMUL_9
#define AO_RCC_CFGR_PLLXTPRE	STM_RCC_CFGR_PLLXTPRE_1
#define AO_RCC_CFGR_PPRE2_DIV	STM_RCC_CFGR_PPRE2_DIV_1
#define AO_RCC_CFGR_PPRE1_DIV	STM_RCC_CFGR_PPRE1_DIV_2
#define AO_RCC_CFGR_HPRE_DIV	STM_RCC_CFGR_HPRE_DIV_1
#define AO_RCC_CFGR_ADCPRE	STM_RCC_CFGR_ADCPRE_6

#include <ao_flash_stm_pins.h>

#define AO_BOOT_PIN		1
#define AO_BOOT_APPLICATION_GPIO	stm_gpioa
#define AO_BOOT_APPLICATION_PIN		0
#define AO_BOOT_APPLICATION_VALUE	1
#define AO_BOOT_APPLICATION_MODE	AO_EXTI_MODE_PULL_UP

#define HAS_USB_PULLUP		0

#endif /* _AO_PINS_H_ */
