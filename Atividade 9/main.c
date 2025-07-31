#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
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
#include "esp_timer.h"
#include "rom/ets_sys.h"

// ===== BIBLIOTECA DO LCD I2C =====
// PINOS DO PCF8475
#define RS 0
#define RW 1
#define EN 2
#define BL 3

// MEDIDAS DOS DISPLAYS
#define DISPLAY_16X02 0
#define DISPLAY_20X04 1

// INSTRUÇÕES DO DISPLAY LCD
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
    else if (lcd->size == DISPLAY_20X04) {
        switch (row) {
            case 0: lcd_i2c_write(lcd, 0, 0x80 + column); break;
            case 1: lcd_i2c_write(lcd, 0, 0x80 + 0x40 + column); break;
            case 2: lcd_i2c_write(lcd, 0, 0x80 + 0x14 + column); break;
            case 3: lcd_i2c_write(lcd, 0, 0x80 + 0x54 + column); break;
            default: break;
        }
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

// ===== DEFINIÇÕES DE PINOS =====
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

// ===== CONFIGURAÇÕES DO SISTEMA =====
#define DEFAULT_ALARM_TEMP 25.0
#define TEMP_INCREMENT 5.0
#define DEBOUNCE_TIME_MS 50
#define TEMP_READ_INTERVAL_MS 500
#define LOG_INTERVAL_MS 5000

// Configurações PWM
#define BUZZER_FREQ 1000
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_DUTY 512

// Configurações I2C LCD
#define I2C_MASTER_PORT I2C_NUM_0
#define LCD_ADDR 0x27

// Configurações SDCard
#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT "/temperature_log.txt"

// ===== MÁQUINA DE ESTADO =====
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_READ_TEMPERATURE,
    STATE_BUTTON_PROCESS,
    STATE_UPDATE_DISPLAY,
    STATE_UPDATE_LEDS,
    STATE_CONTROL_BUZZER,
    STATE_LOG_SDCARD,
    STATE_ERROR
} system_state_e;

typedef enum {
    EVENT_NONE = 0,
    EVENT_TIMER_READ_TEMP,
    EVENT_BUTTON_PRESSED,
    EVENT_TEMP_CHANGED,
    EVENT_ALARM_CHANGED,
    EVENT_TIMER_LOG,
    EVENT_ERROR
} system_event_e;

// ===== ESTRUTURAS GLOBAIS =====
typedef struct {
    float current_temp;
    float alarm_temp;
    float previous_temp;
    float previous_alarm;
    bool alarm_active;
    bool sdcard_ready;
    uint32_t log_counter;
    uint32_t last_button_time[2];
    system_state_e current_state;
    system_event_e pending_event;
} system_data_t;

// ===== VARIÁVEIS GLOBAIS =====
static const char *TAG = "TEMP_FSM";
static esp_adc_cal_characteristics_t adc_chars;
static QueueHandle_t button_queue;
static QueueHandle_t event_queue;
static sdmmc_card_t *card;
static esp_timer_handle_t temp_read_timer;
static esp_timer_handle_t log_timer;

static lcd_i2c_handle_t lcd_handle = {
    .address = LCD_ADDR,
    .num = I2C_MASTER_PORT,
    .backlight = 1,
    .size = DISPLAY_16X02
};

static system_data_t sys_data = {
    .current_temp = 0.0,
    .alarm_temp = DEFAULT_ALARM_TEMP,
    .previous_temp = -999.0,
    .previous_alarm = -999.0,
    .alarm_active = false,
    .sdcard_ready = false,
    .log_counter = 0,
    .last_button_time = {0, 0},
    .current_state = STATE_INIT,
    .pending_event = EVENT_NONE
};

// ===== PROTÓTIPOS DAS FUNÇÕES =====
void init_hardware(void);
void init_adc(void);
void init_pwm(void);
void init_i2c_lcd(void);
void init_buttons(void);
void init_leds(void);
bool init_sdcard(void);
void init_timers(void);
float read_ntc_temperature(void);
void update_led_status(float temp_diff);
void update_lcd_display(void);
void control_buzzer(bool activate);
void log_temperature_to_sdcard(float temperature, float alarm_temp);
void IRAM_ATTR button_isr_handler(void *arg);
void temp_read_timer_callback(void *arg);
void log_timer_callback(void *arg);

// Estados da máquina
void state_init(void);
void state_idle(void);
void state_read_temperature(void);
void state_button_process(void);
void state_update_display(void);
void state_update_leds(void);
void state_control_buzzer(void);
void state_log_sdcard(void);
void state_error(void);

// ===== IMPLEMENTAÇÃO DOS TIMERS =====
void temp_read_timer_callback(void *arg) {
    system_event_e event = EVENT_TIMER_READ_TEMP;
    xQueueSendFromISR(event_queue, &event, NULL);
}

void log_timer_callback(void *arg) {
    system_event_e event = EVENT_TIMER_LOG;
    xQueueSendFromISR(event_queue, &event, NULL);
}

void init_timers(void) {
    // Timer para leitura de temperatura
    esp_timer_create_args_t temp_timer_args = {
        .callback = &temp_read_timer_callback,
        .name = "temp_read_timer"
    };
    esp_timer_create(&temp_timer_args, &temp_read_timer);
    esp_timer_start_periodic(temp_read_timer, TEMP_READ_INTERVAL_MS * 1000);

    // Timer para logging
    esp_timer_create_args_t log_timer_args = {
        .callback = &log_timer_callback,
        .name = "log_timer"
    };
    esp_timer_create(&log_timer_args, &log_timer);
    esp_timer_start_periodic(log_timer, LOG_INTERVAL_MS * 1000);

    ESP_LOGI(TAG, "Timers inicializados");
}

// ===== IMPLEMENTAÇÃO DO HARDWARE =====
void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, 
                           ADC_WIDTH_BIT_12, 1100, &adc_chars);
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
    lcd_i2c_print(&lcd_handle, "Sistema FSM");
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

    button_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_PIN, button_isr_handler, (void *)BUTTON_A_PIN);
    gpio_isr_handler_add(BUTTON_B_PIN, button_isr_handler, (void *)BUTTON_B_PIN);
    ESP_LOGI(TAG, "Botões inicializados");
}

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

bool init_sdcard(void) {
    esp_err_t ret;
    
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
        ESP_LOGE(TAG, "Falha ao inicializar SPI: %s", esp_err_to_name(ret));
        return false;
    }
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SDCARD_CS_PIN;
    slot_config.host_id = host.slot;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao montar SDCard: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Criar arquivo de log se não existir
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
    
    ESP_LOGI(TAG, "SDCard inicializado");
    return true;
}

void init_hardware(void) {
    init_adc();
    init_pwm();
    init_i2c_lcd();
    init_buttons();
    init_leds();
    sys_data.sdcard_ready = init_sdcard();
    init_timers();
}

// ===== ISR E FUNÇÕES DE HARDWARE =====
void IRAM_ATTR button_isr_handler(void *arg) {
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(button_queue, &pin, NULL);
    system_event_e event = EVENT_BUTTON_PRESSED;
    xQueueSendFromISR(event_queue, &event, NULL);
}

float read_ntc_temperature(void) {
    uint32_t adc_reading = 0;
    for (int i = 0; i < 64; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_0);
    }
    adc_reading /= 64;
    
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    float resistance = (3300.0 * 10000.0) / voltage - 10000.0;
    float temperature = 1.0 / (1.0/298.15 + (1.0/3950.0) * log(resistance/10000.0)) - 273.15;
    
    return temperature;
}

void update_led_status(float temp_diff) {
    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);
    gpio_set_level(LED3_PIN, 0);
    gpio_set_level(LED4_PIN, 0);
    
    if (temp_diff <= 0) {
        // Piscar todos os LEDs
        static bool led_state = false;
        led_state = !led_state;
        gpio_set_level(LED1_PIN, led_state);
        gpio_set_level(LED2_PIN, led_state);
        gpio_set_level(LED3_PIN, led_state);
        gpio_set_level(LED4_PIN, led_state);
    } else {
        if (temp_diff <= 20.0) gpio_set_level(LED1_PIN, 1);
        if (temp_diff <= 15.0) gpio_set_level(LED2_PIN, 1);
        if (temp_diff <= 10.0) gpio_set_level(LED3_PIN, 1);
        if (temp_diff <= 2.0) gpio_set_level(LED4_PIN, 1);
    }
}

void update_lcd_display(void) {
    char line1[17], line2[17];
    
    snprintf(line1, 17, "Temp: %.1f%cC", sys_data.current_temp, 0xDF);
    snprintf(line2, 17, "Alarm:%.1f%cC", sys_data.alarm_temp, 0xDF);
    
    lcd_i2c_cursor_set(&lcd_handle, 0, 0);
    lcd_i2c_print(&lcd_handle, "                ");
    lcd_i2c_cursor_set(&lcd_handle, 0, 0);
    lcd_i2c_print(&lcd_handle, line1);
    
    lcd_i2c_cursor_set(&lcd_handle, 0, 1);
    lcd_i2c_print(&lcd_handle, "                ");
    lcd_i2c_cursor_set(&lcd_handle, 0, 1);
    lcd_i2c_print(&lcd_handle, line2);
}

void control_buzzer(bool activate) {
    if (activate) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BUZZER_DUTY);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void log_temperature_to_sdcard(float temperature, float alarm_temp) {
    if (!sys_data.sdcard_ready) return;
    
    FILE* f = fopen(LOG_FILE_PATH, "a");
    if (f == NULL) return;
    
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    const char* status = "NORMAL";
    if (sys_data.alarm_active) {
        status = "ALARM";
    } else {
        float temp_diff = alarm_temp - temperature;
        if (temp_diff <= 2.0) status = "CRITICAL";
        else if (temp_diff <= 10.0) status = "WARNING";
        else if (temp_diff <= 15.0) status = "CAUTION";
        else if (temp_diff <= 20.0) status = "ALERT";
    }
    
    fprintf(f, "%s,%.2f,%.2f,%s\n", timestamp, temperature, alarm_temp, status);
    fclose(f);
    
    sys_data.log_counter++;
    ESP_LOGI(TAG, "Log #%lu salvo: %.2f°C [%s]", sys_data.log_counter, temperature, status);
}

// ===== IMPLEMENTAÇÃO DOS ESTADOS =====
void state_init(void) {
    ESP_LOGI(TAG, "Estado: INIT");
    init_hardware();
    sys_data.current_state = STATE_IDLE;
}

void state_idle(void) {
    // Aguarda eventos
    system_event_e event;
    if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(10)) == pdTRUE) {
        switch (event) {
            case EVENT_TIMER_READ_TEMP:
                sys_data.current_state = STATE_READ_TEMPERATURE;
                break;
            case EVENT_BUTTON_PRESSED:
                sys_data.current_state = STATE_BUTTON_PROCESS;
                break;
            case EVENT_TIMER_LOG:
                sys_data.current_state = STATE_LOG_SDCARD;
                break;
            default:
                break;
        }
    }
}

void state_read_temperature(void) {
    ESP_LOGI(TAG, "Estado: READ_TEMPERATURE");
    
    sys_data.previous_temp = sys_data.current_temp;
    sys_data.current_temp = read_ntc_temperature();
    
    ESP_LOGI(TAG, "Temperatura lida: %.2f°C", sys_data.current_temp);
    
    // Sempre atualiza display e LEDs após leitura
    sys_data.current_state = STATE_UPDATE_DISPLAY;
}

void state_button_process(void) {
    ESP_LOGI(TAG, "Estado: BUTTON_PROCESS");
    
    uint32_t pin;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    while (xQueueReceive(button_queue, &pin, 0) == pdTRUE) {
        int button_index = (pin == BUTTON_A_PIN) ? 0 : 1;
        
        // Debounce por software
        if (current_time - sys_data.last_button_time[button_index] > DEBOUNCE_TIME_MS) {
            sys_data.last_button_time[button_index] = current_time;
            sys_data.previous_alarm = sys_data.alarm_temp;
            
            if (pin == BUTTON_A_PIN) {
                sys_data.alarm_temp += TEMP_INCREMENT;
                ESP_LOGI(TAG, "Botão A: Alarme = %.1f°C", sys_data.alarm_temp);
            } else if (pin == BUTTON_B_PIN) {
                sys_data.alarm_temp -= TEMP_INCREMENT;
                ESP_LOGI(TAG, "Botão B: Alarme = %.1f°C", sys_data.alarm_temp);
            }
        }
    }
    
    sys_data.current_state = STATE_UPDATE_DISPLAY;
}

void state_update_display(void) {
    ESP_LOGI(TAG, "Estado: UPDATE_DISPLAY");
    
    // Sempre atualiza o display
    update_lcd_display();
    
    sys_data.current_state = STATE_UPDATE_LEDS;
}

void state_update_leds(void) {
    ESP_LOGI(TAG, "Estado: UPDATE_LEDS");
    
    float temp_diff = sys_data.alarm_temp - sys_data.current_temp;
    update_led_status(temp_diff);
    
    sys_data.current_state = STATE_CONTROL_BUZZER;
}

void state_control_buzzer(void) {
    ESP_LOGI(TAG, "Estado: CONTROL_BUZZER");
    
    bool should_alarm = sys_data.current_temp >= sys_data.alarm_temp;
    
    if (should_alarm != sys_data.alarm_active) {
        sys_data.alarm_active = should_alarm;
        control_buzzer(should_alarm);
        ESP_LOGI(TAG, "Buzzer: %s", should_alarm ? "ATIVO" : "INATIVO");
    }
    
    sys_data.current_state = STATE_IDLE;
}

void state_log_sdcard(void) {
    ESP_LOGI(TAG, "Estado: LOG_SDCARD");
    
    log_temperature_to_sdcard(sys_data.current_temp, sys_data.alarm_temp);
    
    sys_data.current_state = STATE_IDLE;
}

void state_error(void) {
    ESP_LOGE(TAG, "Estado: ERROR - Reiniciando sistema");
    
    // Piscar todos os LEDs rapidamente
    for (int i = 0; i < 10; i++) {
        gpio_set_level(LED1_PIN, i % 2);
        gpio_set_level(LED2_PIN, i % 2);
        gpio_set_level(LED3_PIN, i % 2);
        gpio_set_level(LED4_PIN, i % 2);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Reiniciar para estado inicial
    sys_data.current_state = STATE_INIT;
}

// ===== TASK PRINCIPAL DA MÁQUINA DE ESTADO =====
void fsm_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando Máquina de Estado");
    
    while (1) {
        switch (sys_data.current_state) {
            case STATE_INIT:
                state_init();
                break;
            case STATE_IDLE:
                state_idle();
                break;
            case STATE_READ_TEMPERATURE:
                state_read_temperature();
                break;
            case STATE_BUTTON_PROCESS:
                state_button_process();
                break;
            case STATE_UPDATE_DISPLAY:
                state_update_display();
                break;
            case STATE_UPDATE_LEDS:
                state_update_leds();
                break;
            case STATE_CONTROL_BUZZER:
                state_control_buzzer();
                break;
            case STATE_LOG_SDCARD:
                state_log_sdcard();
                break;
            case STATE_ERROR:
                state_error();
                break;
            default:
                ESP_LOGE(TAG, "Estado inválido: %d", sys_data.current_state);
                sys_data.current_state = STATE_ERROR;
                break;
        }
        
        // Pequeno delay para não saturar CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===== FUNÇÃO PRINCIPAL =====
void app_main(void) {
    ESP_LOGI(TAG, "=== Sistema de Monitoramento com Máquina de Estado ===");
    
    // Criar filas
    event_queue = xQueueCreate(20, sizeof(system_event_e));
    if (event_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila de eventos");
        return;
    }
    
    // Criar task da máquina de estado
    xTaskCreate(fsm_task, "fsm_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Sistema iniciado - Máquina de Estado ativa");
}
