#include <stdio.h>
#include "blink.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"   // <-- needed for gpio functions
#include "esp_log.h"       // <-- needed for ESP_LOGI

static const char *TAG = "example";
static uint8_t s_led_state = 0;
#define BLINK_GPIO GPIO_NUM_2


static void configure_led(void);
static void blink_led(void);

// Implementation of blink class
void blink::blink_init(void) {
    configure_led();
}

void blink::blink_toggle(void) {
    s_led_state = !s_led_state;
    blink::state = s_led_state;
    blink_led();   // call the static helper
}

// Local helper functions
static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}
