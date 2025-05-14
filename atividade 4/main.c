
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// GPIO definitions
#define LED0 GPIO_NUM_2
#define LED1 GPIO_NUM_3
#define LED2 GPIO_NUM_4
#define LED3 GPIO_NUM_5

#define BUTTON_A GPIO_NUM_10
#define BUTTON_B GPIO_NUM_11

// Debounce parameters
#define DEBOUNCE_TIME_US 200000 // 200ms debounce

// Global variables
static uint8_t counter = 0;
static uint8_t step = 1;
static int64_t last_press_a = 0;
static int64_t last_press_b = 0;

void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);
}

void app_main(void) {
    // Config LEDs
    gpio_config_t io_conf_led = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_led);

    // Config Buttons
    gpio_config_t io_conf_btn = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_A) | (1ULL << BUTTON_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_btn);

    while (1) {
        int64_t now = esp_timer_get_time(); // Current time in µs

        // Handle Button A
        if (gpio_get_level(BUTTON_A) == 1 && (now - last_press_a) > DEBOUNCE_TIME_US) {
            last_press_a = now;
            counter = (counter + step) & 0x0F; // Circular 4-bit counter
            update_leds(counter);
        }

        // Handle Button B
        if (gpio_get_level(BUTTON_B) == 1 && (now - last_press_b) > DEBOUNCE_TIME_US) {
            last_press_b = now;
            step = (step == 1) ? 2 : 1;
        }

        // Short delay to avoid busy loop (not a debounce)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
