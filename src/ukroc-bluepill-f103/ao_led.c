/*
 * Blue Pill PC13 LED is active low. The shared AltOS LED driver assumes an
 * active-high output, so this target provides the small local implementation
 * needed by ao_panic() and the bootstrap indication in ao_ukroc.c.
 */

#include "ao.h"

void
ao_led_on(AO_LED_TYPE colors)
{
	if (colors & AO_LED_GREEN) {
		ao_gpio_set(LED_0_PORT, LED_0_PIN, 0);
	}
}

void
ao_led_off(AO_LED_TYPE colors)
{
	if (colors & AO_LED_GREEN) {
		ao_gpio_set(LED_0_PORT, LED_0_PIN, 1);
	}
}

void
ao_led_set(AO_LED_TYPE colors)
{
	ao_gpio_set(LED_0_PORT, LED_0_PIN, (colors & AO_LED_GREEN) ? 0 : 1);
}

void
ao_led_set_mask(AO_LED_TYPE colors, AO_LED_TYPE mask)
{
	if (mask & AO_LED_GREEN) {
		ao_gpio_set(LED_0_PORT, LED_0_PIN, (colors & AO_LED_GREEN) ? 0 : 1);
	}
}

void
ao_led_for(AO_LED_TYPE colors, AO_TICK_TYPE ticks)
{
	ao_led_on(colors);
	ao_delay(ticks);
	ao_led_off(colors);
}

void
ao_led_init(void)
{
	/* Start with the LED off: PC13 high. */
	ao_enable_output(LED_0_PORT, LED_0_PIN, 1);
}
