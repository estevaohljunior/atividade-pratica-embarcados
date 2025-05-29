#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lcd_i2c.h"

// GPIO definitions
#define LED0 GPIO_NUM_2
#define LED1 GPIO_NUM_3
#define LED2 GPIO_NUM_4
#define LED3 GPIO_NUM_5

#define PWM_LED GPIO_NUM_6

#define BUTTON_A GPIO_NUM_10
#define BUTTON_B GPIO_NUM_11

// I2C config
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_FREQ_HZ 100000

// Debounce parameters
#define DEBOUNCE_TIME_US 200000 // 200ms debounce

// Global variables
static uint8_t counter = 0;
static uint8_t step = 1;
static int64_t last_press_a = 0;
static int64_t last_press_b = 0;

static lcd_i2c_handle_t lcd;

void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);
}

void update_pwm(uint8_t value) {
    uint32_t duty = (value * (8191)) / 15; // PWM 13-bit resolution
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

void update_lcd(uint8_t value) {
    char buf[20];

    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buf, sizeof(buf), "Hex: 0x%X", value);
    for (char *c = buf; *c; c++) {
        lcd_i2c_write(&lcd, 1, *c);
    }

    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buf, sizeof(buf), "Dec: %d", value);
    for (char *c = buf; *c; c++) {
        lcd_i2c_write(&lcd, 1, *c);
    }
}

static void IRAM_ATTR button_a_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time();
    if ((now - last_press_a) > DEBOUNCE_TIME_US) {
        last_press_a = now;
        counter = (counter + step) & 0x0F;
        update_leds(counter);
        update_pwm(counter);
        update_lcd(counter);
    }
}

static void IRAM_ATTR button_b_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time();
    if ((now - last_press_b) > DEBOUNCE_TIME_US) {
        last_press_b = now;
        step = (step == 1) ? 2 : 1;
    }
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
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

    // Config PWM LED
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_LED,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    // Config Buttons
    gpio_config_t io_conf_btn = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_A) | (1ULL << BUTTON_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_btn);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, button_a_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_B, button_b_isr_handler, NULL);

    // I2C and LCD
    i2c_master_init();
    lcd.address = 0x27;
    lcd.num = I2C_MASTER_NUM;
    lcd.backlight = 1;
    lcd.size = DISPLAY_16X02;
    lcd_i2c_init(&lcd);

    // Initial update
    update_leds(counter);
    update_pwm(counter);
    update_lcd(counter);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Loop principal ocioso
    }
}
