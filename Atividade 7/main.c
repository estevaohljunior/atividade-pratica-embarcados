#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// Definições de pinos
#define NTC_ADC_CHANNEL ADC1_CHANNEL_0  // GPIO1
#define BUTTON_A_PIN 2
#define BUTTON_B_PIN 3
#define BUZZER_PIN 4
#define LED1_PIN 5
#define LED2_PIN 6
#define LED3_PIN 7
#define LED4_PIN 8
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 10

// Configurações do sistema
#define DEFAULT_ALARM_TEMP 25.0
#define TEMP_INCREMENT 5.0
#define DEBOUNCE_TIME_MS 50
#define SAMPLE_RATE_MS 500

// Configurações PWM
#define BUZZER_FREQ 1000
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_DUTY 512

// Configurações I2C LCD
#define I2C_MASTER_PORT I2C_NUM_0
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Estruturas globais
typedef struct {
    float current_temp;
    float alarm_temp;
    bool alarm_active;
    uint32_t last_button_time[2];
} system_state_t;

static system_state_t sys_state = {
    .current_temp = 0.0,
    .alarm_temp = DEFAULT_ALARM_TEMP,
    .alarm_active = false,
    .last_button_time = {0, 0}
};

static const char *TAG = "TEMP_MONITOR";
static esp_adc_cal_characteristics_t adc_chars;
static QueueHandle_t button_queue;

// Protótipos das funções
void init_hardware(void);
void init_adc(void);
void init_pwm(void);
void init_i2c_lcd(void);
void init_buttons(void);
void init_leds(void);
float read_ntc_temperature(void);
void update_led_status(float temp_diff);
void update_lcd_display(void);
void control_buzzer(bool activate);
void IRAM_ATTR button_isr_handler(void *arg);
void button_task(void *pvParameters);
void temperature_task(void *pvParameters);
void alarm_task(void *pvParameters);

// Inicialização do ADC
void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, 
                           ADC_WIDTH_BIT_12, 1100, &adc_chars);
    ESP_LOGI(TAG, "ADC inicializado");
}

// Inicialização do PWM para buzzer
void init_pwm(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BUZZER_RESOLUTION,
        .freq_hz = BUZZER_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
    ESP_LOGI(TAG, "PWM inicializado");
}

// Inicialização I2C e LCD
void init_i2c_lcd(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    
    // Inicialização básica do LCD (comandos específicos do controlador)
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "I2C e LCD inicializados");
}

// Inicialização dos botões com interrupção
void init_buttons(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_A_PIN) | (1ULL << BUTTON_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    button_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_PIN, button_isr_handler, (void *)BUTTON_A_PIN);
    gpio_isr_handler_add(BUTTON_B_PIN, button_isr_handler, (void *)BUTTON_B_PIN);
    ESP_LOGI(TAG, "Botões inicializados");
}

// Inicialização dos LEDs
void init_leds(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED1_PIN) | (1ULL << LED2_PIN) | 
                       (1ULL << LED3_PIN) | (1ULL << LED4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "LEDs inicializados");
}

// ISR para tratamento de botões
void IRAM_ATTR button_isr_handler(void *arg) {
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(button_queue, &pin, NULL);
}

// Leitura da temperatura do NTC
float read_ntc_temperature(void) {
    uint32_t adc_reading = 0;
    // Média de múltiplas leituras para estabilidade
    for (int i = 0; i < 64; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_0);
    }
    adc_reading /= 64;
    
    // Conversão para tensão
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    
    // Conversão NTC para temperatura (equação de Steinhart-Hart simplificada)
    // Assumindo NTC 10k com beta = 3950K
    float resistance = (3300.0 * 10000.0) / voltage - 10000.0;
    float temperature = 1.0 / (1.0/298.15 + (1.0/3950.0) * log(resistance/10000.0)) - 273.15;
    
    return temperature;
}

// Atualização do status dos LEDs baseado na diferença de temperatura
void update_led_status(float temp_diff) {
    // Desliga todos os LEDs primeiro
    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);
    gpio_set_level(LED3_PIN, 0);
    gpio_set_level(LED4_PIN, 0);
    
    if (temp_diff <= 0) {
        // Temperatura igual ou acima do alarme - piscar todos os LEDs
        static bool led_state = false;
        led_state = !led_state;
        gpio_set_level(LED1_PIN, led_state);
        gpio_set_level(LED2_PIN, led_state);
        gpio_set_level(LED3_PIN, led_state);
        gpio_set_level(LED4_PIN, led_state);
    } else {
        // LEDs baseados na proximidade do alarme
        if (temp_diff <= 20.0) gpio_set_level(LED1_PIN, 1);
        if (temp_diff <= 15.0) gpio_set_level(LED2_PIN, 1);
        if (temp_diff <= 10.0) gpio_set_level(LED3_PIN, 1);
        if (temp_diff <= 2.0) gpio_set_level(LED4_PIN, 1);
    }
}

// Atualização do display LCD
void update_lcd_display(void) {
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "Temp: %.1f C", sys_state.current_temp);
    snprintf(line2, sizeof(line2), "Alarm: %.1f C", sys_state.alarm_temp);
    
    // Aqui você implementaria os comandos específicos para seu LCD
    // Exemplo simplificado:
    ESP_LOGI(TAG, "LCD - %s | %s", line1, line2);
}

// Controle do buzzer
void control_buzzer(bool activate) {
    if (activate) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BUZZER_DUTY);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Task para tratamento de botões com debounce
void button_task(void *pvParameters) {
    uint32_t pin;
    uint32_t current_time;
    
    while (1) {
        if (xQueueReceive(button_queue, &pin, portMAX_DELAY)) {
            current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Debounce por software
            int button_index = (pin == BUTTON_A_PIN) ? 0 : 1;
            if (current_time - sys_state.last_button_time[button_index] > DEBOUNCE_TIME_MS) {
                sys_state.last_button_time[button_index] = current_time;
                
                if (pin == BUTTON_A_PIN) {
                    sys_state.alarm_temp += TEMP_INCREMENT;
                    ESP_LOGI(TAG, "Botão A: Alarme = %.1f°C", sys_state.alarm_temp);
                } else if (pin == BUTTON_B_PIN) {
                    sys_state.alarm_temp -= TEMP_INCREMENT;
                    ESP_LOGI(TAG, "Botão B: Alarme = %.1f°C", sys_state.alarm_temp);
                }
                update_lcd_display();
            }
        }
    }
}

// Task para leitura de temperatura
void temperature_task(void *pvParameters) {
    while (1) {
        sys_state.current_temp = read_ntc_temperature();
        
        float temp_diff = sys_state.alarm_temp - sys_state.current_temp;
        update_led_status(temp_diff);
        update_lcd_display();
        
        ESP_LOGI(TAG, "Temperatura: %.1f°C, Alarme: %.1f°C", 
                sys_state.current_temp, sys_state.alarm_temp);
        
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_RATE_MS));
    }
}

// Task para controle do alarme
void alarm_task(void *pvParameters) {
    while (1) {
        bool should_alarm = sys_state.current_temp >= sys_state.alarm_temp;
        
        if (should_alarm != sys_state.alarm_active) {
            sys_state.alarm_active = should_alarm;
            control_buzzer(should_alarm);
            ESP_LOGI(TAG, "Alarme: %s", should_alarm ? "ATIVO" : "INATIVO");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Inicialização geral do hardware
void init_hardware(void) {
    init_adc();
    init_pwm();
    init_i2c_lcd();
    init_buttons();
    init_leds();
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Sistema de Monitoramento de Temperatura");
    
    // Inicialização do hardware
    init_hardware();
    
    // Criação das tasks
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(temperature_task, "temperature_task", 2048, NULL, 4, NULL);
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "Sistema inicializado com sucesso!");
}
