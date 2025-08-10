#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

// ===== BIBLIOTECA DO LCD I2C =====
#define RS 0
#define RW 1
#define EN 2
#define BL 3
#define DISPLAY_16X02 0
#define DISPLAY_20X04 1


#define CLEAR_DISPLAY               0x01
#define RETURN_HOME_UNSHIFT         0x02
#define CURSOR_RIGHT_NO_SHIFT       0x04
#define CURSOR_RIGHT_SHIFT          0x05
#define CURSOR_RIGHT_NO_SHIFT_LEFT  0x06
#define CURSOR_RIGHT_SHIFT_LEFT     0x07
#define DISPLAY_OFF                 0x08
#define DISPLAY_ON_CURSOR_OFF       0x0C
#define DISPLAY_ON_CURSOR_ON_STEADY 0x0E
#define DISPLAY_ON_CURSOR_ON_BLINK  0x0F
#define RETURN_HOME                 0x80
#define SHIFT_CURSOR_LEFT           0x10
#define SHIFT_CURSOR_RIGHT          0x14
#define SHIFT_DISPLAY_LEFT          0x18
#define SHIFT_DISPLAY_RIGHT         0x1C
#define SET_4BIT_MODE               0x28

typedef struct {
    uint8_t address;
    uint8_t num;
    uint8_t backlight;
    uint8_t size;
} lcd_i2c_handle_t;

void i2c_write_byte(lcd_i2c_handle_t * lcd, uint8_t data) {
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (lcd->address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(i2c_cmd, data, 1);
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(lcd->num, i2c_cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(i2c_cmd);
}

void lcd_i2c_write(lcd_i2c_handle_t * lcd, char rs_flag, char data_byte) {
    uint8_t buffer_byte = ((1 << RS) * rs_flag) | ((1 << BL) * lcd->backlight);

    buffer_byte |= (buffer_byte & 0x0F) | (0xF0 & data_byte);
    buffer_byte |= (1 << EN);
    i2c_write_byte(lcd, buffer_byte);
    ets_delay_us(10);
    buffer_byte &= ~(1 << EN);
    i2c_write_byte(lcd, buffer_byte);
    ets_delay_us(50);

    buffer_byte = (buffer_byte & 0x0F) | (data_byte << 4);
    buffer_byte |= (1 << EN);
    i2c_write_byte(lcd, buffer_byte);
    ets_delay_us(10);
    buffer_byte &= ~(1 << EN);
    i2c_write_byte(lcd, buffer_byte);
    ets_delay_us(50);

    if (data_byte == CLEAR_DISPLAY) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void lcd_i2c_init(lcd_i2c_handle_t * lcd) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_i2c_write(lcd, 0, RETURN_HOME_UNSHIFT);
    lcd_i2c_write(lcd, 0, SET_4BIT_MODE);
    lcd_i2c_write(lcd, 0, CLEAR_DISPLAY);
    lcd_i2c_write(lcd, 0, DISPLAY_ON_CURSOR_OFF);
    lcd_i2c_write(lcd, 0, CURSOR_RIGHT_NO_SHIFT_LEFT);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_i2c_cursor_set(lcd_i2c_handle_t * lcd, uint8_t column, uint8_t row) {
    if (lcd->size == DISPLAY_16X02) {
        if (row) lcd_i2c_write(lcd, 0, 0x80 + 0x40 + column);
        else lcd_i2c_write(lcd, 0, 0x80 + column);
    }
}

void lcd_i2c_print(lcd_i2c_handle_t * lcd, const char * format_string, ...) {
    uint16_t i = 0;
    char buffer_string[128];

    va_list arguments;
    va_start(arguments, format_string);
    vsnprintf(buffer_string, sizeof(buffer_string), format_string, arguments);
    va_end(arguments);

    while (buffer_string[i] != '\0') {
        lcd_i2c_write(lcd, 1, buffer_string[i]);
        i++;
    }
}
// ===== FIM DA BIBLIOTECA DO LCD =====


#define NTC_ADC_CHANNEL ADC_CHANNEL_0   
#define BUTTON_A_PIN 2                   
#define BUTTON_B_PIN 3                   
#define BUZZER_PIN 4                     


#define LED0_PIN 5                       
#define LED1_PIN 6                       
#define LED2_PIN 7                       
#define LED3_PIN 8                       

// LCD I2C
#define I2C_SDA_PIN 9                    
#define I2C_SCL_PIN 10                   

// MicroSD SPI 
#define SD_DI_PIN 11                     
#define SD_SCK_PIN 12                    
#define SD_DO_PIN 13                     
#define SD_CS_PIN 14                     

// ===== CONFIGURAÇÕES =====
#define DEFAULT_ALARM_TEMP 25.0
#define TEMP_INCREMENT 5.0
#define DEBOUNCE_TIME_MS 50
#define TEMP_READ_INTERVAL_MS 500
#define LCD_UPDATE_INTERVAL_MS 250
#define LED_UPDATE_INTERVAL_MS 100
#define LOG_INTERVAL_MS 5000

#define BUZZER_FREQ 1000
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_DUTY 512

#define I2C_MASTER_PORT I2C_NUM_0
#define LCD_ADDR 0x27

#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT "/temperature_log.txt"

// ===== ESTRUTURAS GLOBAIS =====
typedef struct {
    float current_temp;
    float alarm_temp;
    bool alarm_active;
    bool temp_updated;
    bool alarm_updated;
    bool sdcard_ready;
    uint32_t log_counter;
    uint32_t last_button_time[2];
} system_data_t;

// ===== VARIÁVEIS GLOBAIS =====
static const char *TAG = "TEMP_FREERTOS";


static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;


static QueueHandle_t button_queue;
static QueueHandle_t temp_queue;
static QueueHandle_t alarm_queue;
static QueueHandle_t log_queue;
static SemaphoreHandle_t data_mutex;

// Inicializar dados com valores padrão
static system_data_t sys_data = {
    .current_temp = 25.0,           // Valor inicial válido
    .alarm_temp = DEFAULT_ALARM_TEMP,
    .alarm_active = false,
    .temp_updated = false,
    .alarm_updated = false,
    .sdcard_ready = false,
    .log_counter = 0,
    .last_button_time = {0, 0}
};

static sdmmc_card_t *card;
static lcd_i2c_handle_t lcd_handle = {
    .address = LCD_ADDR,
    .num = I2C_MASTER_PORT,
    .backlight = 1,
    .size = DISPLAY_16X02
};

// ===== PROTÓTIPOS =====
void init_hardware(void);
void init_adc(void);
void init_pwm(void);
void init_i2c_lcd(void);
void init_buttons(void);
void init_leds(void);
bool init_sdcard(void);
float read_ntc_temperature(void);
void update_leds_status(float temp_diff);
// Remover IRAM_ATTR conflitante do protótipo
void button_isr_handler(void *arg);

// Tasks do FreeRTOS
void temperature_read_task(void *pvParameters);
void button_process_task(void *pvParameters);
void buzzer_control_task(void *pvParameters);
void lcd_update_task(void *pvParameters);
void led_control_task(void *pvParameters);
void sdcard_log_task(void *pvParameters);

// ===== IMPLEMENTAÇÃO DO HARDWARE =====

// CORREÇÃO 5: Nova implementação do ADC
void init_adc(void) {
    // Configuração do ADC OneShot
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "inicializar ADC OneShot: %s", esp_err_to_name(ret));
        return;
    }

    // Configuração do canal ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,  // Nova nomenclatura
    };
    ret = adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "configurar canal ADC: %s", esp_err_to_name(ret));
        return;
    }

    // Calibração do ADC
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = NTC_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibrado com sucesso");
    } else {
        ESP_LOGW(TAG, "Calibração ADC");
        cali_handle = NULL;
    }

    ESP_LOGI(TAG, "ADC inicializado");
}

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
    
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_i2c_init(&lcd_handle);
    
    lcd_i2c_cursor_set(&lcd_handle, 0, 0);
    lcd_i2c_print(&lcd_handle, "FreeRTOS System");
    lcd_i2c_cursor_set(&lcd_handle, 0, 1);
    lcd_i2c_print(&lcd_handle, "Inicializando...");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "LCD I2C inicializado");
}

void init_buttons(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_A_PIN) | (1ULL << BUTTON_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_PIN, button_isr_handler, (void *)BUTTON_A_PIN);
    gpio_isr_handler_add(BUTTON_B_PIN, button_isr_handler, (void *)BUTTON_B_PIN);
    ESP_LOGI(TAG, "Botões inicializados");
}

void init_leds(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED0_PIN) | (1ULL << LED1_PIN) | 
                       (1ULL << LED2_PIN) | (1ULL << LED3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Apagar todos os LEDs inicialmente
    gpio_set_level(LED0_PIN, 0);
    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);
    gpio_set_level(LED3_PIN, 0);
    
    ESP_LOGI(TAG, "LEDs inicializados");
}

bool init_sdcard(void) {
    esp_err_t ret;
    
    // Configurar SPI para MicroSD
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_DI_PIN,
        .miso_io_num = SD_DO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI: %s", esp_err_to_name(ret));
        return false;
    }
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MicroSD: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Criar arquivo de log
    FILE* f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) {
        f = fopen(LOG_FILE_PATH, "w");
        if (f != NULL) {
            fprintf(f, "Timestamp,Temperature(C),Alarm_Temp(C),Status\n");
            fclose(f);
        }
    } else {
        fclose(f);
    }
    
    ESP_LOGI(TAG, "MicroSD inicializado");
    return true;
}

void init_hardware(void) {
    // Criar mutex para proteção de dados
    data_mutex = xSemaphoreCreateMutex();
    
    // Criar filas
    button_queue = xQueueCreate(10, sizeof(uint32_t));
    temp_queue = xQueueCreate(5, sizeof(float));
    alarm_queue = xQueueCreate(5, sizeof(bool));
    log_queue = xQueueCreate(20, sizeof(float));
    
    // Inicializar hardware
    init_adc();
    init_pwm();
    init_i2c_lcd();
    init_buttons();
    init_leds();
    sys_data.sdcard_ready = init_sdcard();
}

// ===== ISR E FUNÇÕES DE HARDWARE =====
// Manter IRAM_ATTR apenas na implementação
void IRAM_ATTR button_isr_handler(void *arg) {
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(button_queue, &pin, NULL);
}

// Nova implementação de leitura do ADC
float read_ntc_temperature(void) {
    int adc_reading = 0;
    esp_err_t ret;
    
    // Fazer múltiplas leituras para estabilidade
    for (int i = 0; i < 64; i++) {
        int raw_value;
        ret = adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &raw_value);
        if (ret == ESP_OK) {
            adc_reading += raw_value;
        }
    }
    adc_reading /= 64;
    
    // Converter para tensão
    int voltage = 0;
    if (cali_handle != NULL) {
        ret = adc_cali_raw_to_voltage(cali_handle, adc_reading, &voltage);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "calibração ADC, usando cálculo manual");
            voltage = adc_reading * 3300 / 4095; // Cálculo manual para 12-bit
        }
    } else {
        voltage = adc_reading * 3300 / 4095; // Cálculo manual para 12-bit
    }
    
    // Conversão NTC para temperatura (equação de Steinhart-Hart simplificada)
    float resistance = (3300.0 * 10000.0) / voltage - 10000.0;
    float temperature = 1.0 / (1.0/298.15 + (1.0/3950.0) * log(resistance/10000.0)) - 273.15;
    
    return temperature;
}

void update_leds_status(float temp_diff) {
    // Apagar todos os LEDs primeiro
    gpio_set_level(LED0_PIN, 0);
    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);
    gpio_set_level(LED3_PIN, 0);
    
    if (temp_diff <= 0) {
        // Temperatura >= alarme: piscar todos os LEDs
        static bool led_state = false;
        led_state = !led_state;
        gpio_set_level(LED0_PIN, led_state);
        gpio_set_level(LED1_PIN, led_state);
        gpio_set_level(LED2_PIN, led_state);
        gpio_set_level(LED3_PIN, led_state);
    } else {
        // LEDs baseados na proximidade do alarme
        if (temp_diff <= 20.0) gpio_set_level(LED0_PIN, 1);  // LED0 = 20°C
        if (temp_diff <= 15.0) gpio_set_level(LED1_PIN, 1);  // LED1 = 15°C
        if (temp_diff <= 10.0) gpio_set_level(LED2_PIN, 1);  // LED2 = 10°C
        if (temp_diff <= 2.0)  gpio_set_level(LED3_PIN, 1);  // LED3 = 2°C
    }
}

// ===== TASKS DO FREERTOS =====

// Thread A: Leitura da temperatura NTC
void temperature_read_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        float temp = read_ntc_temperature();
        
        // Proteger acesso aos dados com mutex
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float old_temp = sys_data.current_temp;
            sys_data.current_temp = temp;
            
            if (fabs(temp - old_temp) > 0.1) { // Mudança significativa
                sys_data.temp_updated = true;
                xQueueSend(temp_queue, &temp, 0);
                xQueueSend(log_queue, &temp, 0);
            }
            
            xSemaphoreGive(data_mutex);
        }
        
        ESP_LOGI(TAG, "Temperatura lida: %.2f°C", temp);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
    }
}

// Thread B: Processamento de botões (Parte A)
void button_process_task(void *pvParameters) {
    uint32_t pin;
    
    while (1) {
        if (xQueueReceive(button_queue, &pin, portMAX_DELAY) == pdTRUE) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                int button_index = (pin == BUTTON_A_PIN) ? 0 : 1;
                
                // Debounce por software
                if (current_time - sys_data.last_button_time[button_index] > DEBOUNCE_TIME_MS) {
                    sys_data.last_button_time[button_index] = current_time;
                    
                    if (pin == BUTTON_A_PIN) {
                        sys_data.alarm_temp += TEMP_INCREMENT;
                        ESP_LOGI(TAG, "Botão A: Alarme = %.1f°C", sys_data.alarm_temp);
                    } else if (pin == BUTTON_B_PIN) {
                        sys_data.alarm_temp -= TEMP_INCREMENT;
                        ESP_LOGI(TAG, "Botão B: Alarme = %.1f°C", sys_data.alarm_temp);
                    }
                    
                    sys_data.alarm_updated = true;
                }
                
                xSemaphoreGive(data_mutex);
            }
        }
    }
}

// Thread C: Controle do buzzer PWM (Parte B)
void buzzer_control_task(void *pvParameters) {
    bool current_alarm_state = false;
    
    while (1) {
        bool should_alarm = false;
        
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            should_alarm = sys_data.current_temp >= sys_data.alarm_temp;
            xSemaphoreGive(data_mutex);
        }
        
        if (should_alarm != current_alarm_state) {
            current_alarm_state = should_alarm;
            
            if (should_alarm) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BUZZER_DUTY);
                ESP_LOGI(TAG, "Buzzer ATIVO");
            } else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ESP_LOGI(TAG, "Buzzer INATIVO");
            }
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            
            // Atualizar estado global
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                sys_data.alarm_active = should_alarm;
                xSemaphoreGive(data_mutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Thread D: Atualização do LCD I2C (Parte C)
void lcd_update_task(void *pvParameters) {
    float last_temp = -999.0;
    float last_alarm = -999.0;
    
    while (1) {
        bool need_update = false;
        // Inicializar variáveis com valores padrão
        float current_temp = 25.0;
        float alarm_temp = DEFAULT_ALARM_TEMP;
        
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_temp = sys_data.current_temp;
            alarm_temp = sys_data.alarm_temp;
            
            if (sys_data.temp_updated || sys_data.alarm_updated) {
                need_update = true;
                sys_data.temp_updated = false;
                sys_data.alarm_updated = false;
            }
            
            xSemaphoreGive(data_mutex);
        }
        
        if (need_update || fabs(current_temp - last_temp) > 0.1 || 
            fabs(alarm_temp - last_alarm) > 0.1) {
            
            char line1[17], line2[17];
            snprintf(line1, 17, "Temp: %.1f%cC", current_temp, 0xDF);
            snprintf(line2, 17, "Alarm:%.1f%cC", alarm_temp, 0xDF);
            
            lcd_i2c_cursor_set(&lcd_handle, 0, 0);
            lcd_i2c_print(&lcd_handle, "                ");
            lcd_i2c_cursor_set(&lcd_handle, 0, 0);
            lcd_i2c_print(&lcd_handle, line1);
            
            lcd_i2c_cursor_set(&lcd_handle, 0, 1);
            lcd_i2c_print(&lcd_handle, "                ");
            lcd_i2c_cursor_set(&lcd_handle, 0, 1);
            lcd_i2c_print(&lcd_handle, line2);
            
            last_temp = current_temp;
            last_alarm = alarm_temp;
            
            ESP_LOGI(TAG, "LCD atualizado: %.1f°C / %.1f°C", current_temp, alarm_temp);
        }
        
        vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS));
    }
}

// Thread E: Controle dos LEDs (Parte D)
void led_control_task(void *pvParameters) {
    while (1) {
        // Inicializar variáveis com valores padrão
        float temp_diff = DEFAULT_ALARM_TEMP;
        
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            temp_diff = sys_data.alarm_temp - sys_data.current_temp;
            xSemaphoreGive(data_mutex);
        }
        
        update_leds_status(temp_diff);
        
        vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS));
    }
}

// Thread F: Logging no SDCard (Parte E)
void sdcard_log_task(void *pvParameters) {
    TickType_t last_log_time = xTaskGetTickCount();
    
    while (1) {
        if (!sys_data.sdcard_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        TickType_t current_time = xTaskGetTickCount();
        
        // Log a cada LOG_INTERVAL_MS
        if ((current_time - last_log_time) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            // Inicializar variáveis com valores padrão
            float current_temp = 25.0;
            float alarm_temp = DEFAULT_ALARM_TEMP;
            bool alarm_active = false;
            
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_temp = sys_data.current_temp;
                alarm_temp = sys_data.alarm_temp;
                alarm_active = sys_data.alarm_active;
                xSemaphoreGive(data_mutex);
            }
            
            FILE* f = fopen(LOG_FILE_PATH, "a");
            if (f != NULL) {
                time_t now;
                struct tm timeinfo;
                time(&now);
                localtime_r(&now, &timeinfo);
                
                char timestamp[64];
                strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
                
                const char* status = "NORMAL";
                if (alarm_active) {
                    status = "ALARM";
                } else {
                    float temp_diff = alarm_temp - current_temp;
                    if (temp_diff <= 2.0) status = "CRITICAL";
                    else if (temp_diff <= 10.0) status = "WARNING";
                    else if (temp_diff <= 15.0) status = "CAUTION";
                    else if (temp_diff <= 20.0) status = "ALERT";
                }
                
                fprintf(f, "%s,%.2f,%.2f,%s\n", timestamp, current_temp, alarm_temp, status);
                fclose(f);
                
                sys_data.log_counter++;
                ESP_LOGI(TAG, "Log #%lu salvo: %.2f°C [%s]", sys_data.log_counter, current_temp, status);
            }
            
            last_log_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===== FUNÇÃO PRINCIPAL =====
void app_main(void) {
    ESP_LOGI(TAG, "=== Sistema FreeRTOS Multi-Thread - Corrigido ===");
    
    // Inicializar hardware e criar estruturas FreeRTOS
    init_hardware();
    
    // Criar tasks do FreeRTOS
    xTaskCreatePinnedToCore(temperature_read_task, "temp_read", 2048, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(button_process_task, "button_proc", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzer_control_task, "buzzer_ctrl", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(lcd_update_task, "lcd_update", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(led_control_task, "led_control", 2048, NULL, 3, NULL, 1);
    
    if (sys_data.sdcard_ready) {
        xTaskCreatePinnedToCore(sdcard_log_task, "sdcard_log", 4096, NULL, 2, NULL, 0);
        ESP_LOGI(TAG, "Task de logging MicroSD criada");
    }
    
    ESP_LOGI(TAG, "Sistema FreeRTOS Multi-Thread iniciado com sucesso!");
    ESP_LOGI(TAG, "Core 0: Temp Read, Button Proc, SDCard Log");
    ESP_LOGI(TAG, "Core 1: Buzzer, LCD, LED Control");
}
