#include "leds.h"
#include "drone_config.h"
#include "driver/gpio.h"

void leds_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ARMED_LED_GPIO) | (1ULL << LOWBAT_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    led_armed_set(false);
    led_lowbat_set(false);
}

void led_armed_set(bool on)
{
    gpio_set_level(ARMED_LED_GPIO, on ? 1 : 0);
}

void led_lowbat_set(bool on)
{
    gpio_set_level(LOWBAT_LED_GPIO, on ? 1 : 0);
}
