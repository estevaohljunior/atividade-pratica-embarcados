#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

// Queue handle for button events
static QueueHandle_t gpio_evt_queue = NULL;

// Interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

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

    // Config Buttons with pull-up and interrupt on rising edge
    gpio_config_t io_conf_btn = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_A) | (1ULL << BUTTON_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_btn);

    // Create a queue to handle gpio event from ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    
    // Hook ISR handler for specific GPIO pins
    gpio_isr_handler_add(BUTTON_A, gpio_isr_handler, (void*) BUTTON_A);
    gpio_isr_handler_add(BUTTON_B, gpio_isr_handler, (void*) BUTTON_B);

    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();
            
            if (io_num == BUTTON_A && (now - last_press_a) > DEBOUNCE_TIME_US) {
                last_press_a = now;
                counter = (counter + step) & 0x0F; // Circular 4-bit counter
                update_leds(counter);
            }
            else if (io_num == BUTTON_B && (now - last_press_b) > DEBOUNCE_TIME_US) {
                last_press_b = now;
                step = (step == 1) ? 2 : 1;
            }
        }
    }
}
