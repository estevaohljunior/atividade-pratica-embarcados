#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_sntp.h"

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

// Pinos SPI para SDCard
#define SDCARD_MOSI_PIN 11
#define SDCARD_CLK_PIN 12
#define SDCARD_MISO_PIN 13
#define SDCARD_CS_PIN 14

// Configurações do sistema
#define DEFAULT_ALARM_TEMP 25.0
#define TEMP_INCREMENT 5.0
#define DEBOUNCE_TIME_MS 50
#define SAMPLE_RATE_MS 500
#define LOG_INTERVAL_MS 5000  // Log a cada 5 segundos

// Configurações PWM
#define BUZZER_FREQ 1000
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_DUTY 512

// Configurações I2C LCD
#define I2C_MASTER_PORT I2C_NUM_0
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Configurações SDCard
#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT "/temperature_log.txt"

// Estruturas globais
typedef struct {
    float current_temp;
    float alarm_temp;
    bool alarm_active;
    uint32_t last_button_time[2];
    bool sdcard_ready;
    uint32_t log_counter;
} system_state_t;

static system_state_t sys_state = {
    .current_temp = 0.0,
    .alarm_temp = DEFAULT_ALARM_TEMP,
    .alarm_active = false,
    .last_button_time = {0, 0},
    .sdcard_ready = false,
    .log_counter = 0
};

static const char *TAG = "TEMP_MONITOR";
static esp_adc_cal_characteristics_t adc_chars;
static QueueHandle_t button_queue;
static sdmmc_card_t *card;

// Protótipos das funções
void init_hardware(void);
void init_adc(void);
void init_pwm(void);
void init_i2c_lcd(void);
void init_buttons(void);
void init_leds(void);
bool init_sdcard(void);
float read_ntc_temperature(void);
void update_led_status(float temp_diff);
void update_lcd_display(void);
void control_buzzer(bool activate);
void log_temperature_to_sdcard(float temperature, float alarm_temp);
void IRAM_ATTR button_isr_handler(void *arg);
void button_task(void *pvParameters);
void temperature_task(void *pvParameters);
void alarm_task(void *pvParameters);
void sdcard_log_task(void *pvParameters);

// Inicialização do SDCard
bool init_sdcard(void) {
    esp_err_t ret;
    
    // Configuração do host SPI
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SDCARD_MOSI_PIN,
        .miso_io_num = SDCARD_MISO_PIN,
        .sclk_io_num = SDCARD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar barramento SPI: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configuração do slot SPI
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SDCARD_CS_PIN;
    slot_config.host_id = host.slot;
    
    // Configuração do sistema de arquivos
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Falha ao montar sistema de arquivos");
        } else {
            ESP_LOGE(TAG, "Falha ao inicializar SDCard: %s", esp_err_to_name(ret));
        }
        return false;
    }
    
    // Informações do SDCard
    sdmmc_card_print_info(stdout, card);
    
    // Criar arquivo de log com cabeçalho se não existir
    FILE* f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) {
        f = fopen(LOG_FILE_PATH, "w");
        if (f != NULL) {
            fprintf(f, "Timestamp,Temperature(°C),Alarm_Temp(°C),Status\n");
            fclose(f);
            ESP_LOGI(TAG, "Arquivo de log criado com sucesso");
        }
    } else {
        fclose(f);
        ESP_LOGI(TAG, "Arquivo de log já existe");
    }
    
    ESP_LOGI(TAG, "SDCard inicializado com sucesso");
    return true;
}

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

// Função para log de temperatura no SDCard
void log_temperature_to_sdcard(float temperature, float alarm_temp) {
    if (!sys_state.sdcard_ready) {
        return;
    }
    
    FILE* f = fopen(LOG_FILE_PATH, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Falha ao abrir arquivo de log");
        return;
    }
    
    // Obter timestamp atual
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    // Determinar status do sistema
    const char* status = "NORMAL";
    if (sys_state.alarm_active) {
        status = "ALARM";
    } else {
        float temp_diff = alarm_temp - temperature;
        if (temp_diff <= 2.0) status = "CRITICAL";
        else if (temp_diff <= 10.0) status = "WARNING";
        else if (temp_diff <= 15.0) status = "CAUTION";
        else if (temp_diff <= 20.0) status = "ALERT";
    }
    
    // Escrever dados no arquivo
    fprintf(f, "%s,%.2f,%.2f,%s\n", timestamp, temperature, alarm_temp, status);
    fclose(f);
    
    sys_state.log_counter++;
    ESP_LOGI(TAG, "Log #%lu: %.2f°C [%s] salvo no SDCard", 
             sys_state.log_counter, temperature, status);
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

// Task para logging no SDCard
void sdcard_log_task(void *pvParameters) {
    TickType_t last_log_time = xTaskGetTickCount();
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Log a cada LOG_INTERVAL_MS
        if ((current_time - last_log_time) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            log_temperature_to_sdcard(sys_state.current_temp, sys_state.alarm_temp);
            last_log_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Verifica a cada 1 segundo
    }
}

// Inicialização geral do hardware
void init_hardware(void) {
    init_adc();
    init_pwm();
    init_i2c_lcd();
    init_buttons();
    init_leds();
    sys_state.sdcard_ready = init_sdcard();
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Sistema de Monitoramento com SDCard");
    
    // Inicialização do hardware
    init_hardware();
    
    // Criação das tasks
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(temperature_task, "temperature_task", 2048, NULL, 4, NULL);
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 3, NULL);
    
    // Task de logging apenas se SDCard estiver disponível
    if (sys_state.sdcard_ready) {
        xTaskCreate(sdcard_log_task, "sdcard_log_task", 4096, NULL, 2, NULL);
        ESP_LOGI(TAG, "Sistema de logging no SDCard ativado");
    } else {
        ESP_LOGW(TAG, "SDCard não disponível - logging desabilitado");
    }
    
    ESP_LOGI(TAG, "Sistema inicializado com sucesso!");
}
