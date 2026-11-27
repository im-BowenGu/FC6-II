/*
 * Minimal Wokwi STM32F103 harness for the real S-LX guidance core.
 *
 * This is NOT the flight image: Wokwi has no BMP280/ICM20689/PCA9685 parts, so
 * the I2C sensors and servo driver are replaced by a scenario-driven voltage
 * source here. The control law itself is the unchanged C core from
 * rockets/s-lx/src/slx_profile.c + slx_aero.c, and telemetry is emitted with
 * the same slx_format_csv_row() used on hardware. The authoritative real-code
 * HIL is the Verilator + shared-memory harness in ../../ (make run).
 */
#include <stdbool.h>
#include <stdint.h>

#include "slx_profile.h"

#define RCC_BASE      0x40021000u
#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x18u))
#define GPIOA_BASE    0x40010800u
#define GPIOA_CRH     (*(volatile uint32_t *)(GPIOA_BASE + 0x04u))
#define GPIOC_BASE    0x40011000u
#define GPIOC_CRH     (*(volatile uint32_t *)(GPIOC_BASE + 0x04u))
#define GPIOC_ODR     (*(volatile uint32_t *)(GPIOC_BASE + 0x0Cu))
#define USART1_BASE   0x40013800u
#define USART1_SR     (*(volatile uint32_t *)(USART1_BASE + 0x00u))
#define USART1_DR     (*(volatile uint32_t *)(USART1_BASE + 0x04u))
#define USART1_BRR    (*(volatile uint32_t *)(USART1_BASE + 0x08u))
#define USART1_CR1    (*(volatile uint32_t *)(USART1_BASE + 0x0Cu))

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

int main(void);
void reset_handler(void);
static void default_handler(void);

typedef void (*isr_t)(void);

/* Cortex-M3 vector table: enough entries to cover the STM32F103 IRQ set. */
__attribute__((section(".isr_vector"), used))
const isr_t vector_table[] = {
    (isr_t)&_estack, reset_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler, default_handler, default_handler,
    default_handler, default_handler
};

static void default_handler(void)
{
    for (;;) { }
}

void reset_handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) { *dst++ = *src++; }
    dst = &_sbss;
    while (dst < &_ebss) { *dst++ = 0u; }
    main();
    for (;;) { }
}

static void uart_init(void)
{
    /* HSI = 8 MHz. PA9 = USART1_TX (AF PP 50 MHz), PA10 = floating input. */
    RCC_APB2ENR |= (1u << 2u) | (1u << 14u);
    GPIOA_CRH = (GPIOA_CRH & ~0x00000FF0u) | (0x4u << 8u) | (0xBu << 4u);
    USART1_BRR = 0x45u;             /* 8 MHz / (16 * 115200) ~= 4.34 */
    USART1_CR1 = (1u << 13u) | (1u << 3u) | (1u << 2u); /* UE, TE, RE */
}

static void puts_uart(const char *s)
{
    while (*s != '\0') {
        while ((USART1_SR & (1u << 7u)) == 0u) { }
        USART1_DR = (uint32_t)(uint8_t)*s++;
    }
}

static void delay_ms(volatile uint32_t ms)
{
    while (ms-- > 0u) {
        volatile uint32_t i;
        for (i = 0u; i < 800u; i++) { }
    }
}

int main(void)
{
    slx_guidance_t g;
    char row[256];
    uint32_t t_ms;

    uart_init();
    /* PC13 status LED: general-purpose push-pull output. */
    RCC_APB2ENR |= (1u << 4u);
    GPIOC_CRH = (GPIOC_CRH & ~0x00F00000u) | (0x1u << 20u);
    GPIOC_ODR |= (1u << 13u);

    puts_uart("S-LX HIL WOKWI BOOT\r\n");
    puts_uart(SLX_CSV_HEADER "\r\n");

    slx_guidance_init(&g);
    slx_guidance_set_sea_level_pressure(&g, 101325.0f);
    slx_guidance_enter_armed(&g);

    for (t_ms = 0u; t_ms <= 300u; t_ms += 10u) {
        slx_imu_t imu = {0};
        float pressure_pa = 101325.0f - (float)t_ms * 2.5f;
        slx_snapshot_t snap;

        /* Scenario substitute: a short boost-phase acceleration pulse so the
         * real launch detector and controller transition through LAUNCH. */
        imu.ax_g = 0.0f;
        imu.ay_g = 0.0f;
        imu.az_g = (t_ms >= 50u && t_ms < 200u) ? 4.0f : 1.0f;
        imu.gz_dps = 0.0f;

        snap = slx_guidance_tick(&g, t_ms, true, pressure_pa, imu);
        slx_format_csv_row(row, sizeof(row), &snap, 3.9f);
        puts_uart(row);

        if ((t_ms / 10u) % 5u == 0u) {
            GPIOC_ODR ^= (1u << 13u);
        }
        delay_ms(1u);
    }
    puts_uart("S-LX HIL WOKWI DONE\r\n");
    GPIOC_ODR &= ~(1u << 13u);
    for (;;) { }
}
