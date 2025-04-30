#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Definição dos pinos GPIO para os LEDs
#define LED_1 2  // LED que pisca a cada 1000ms
#define LED_2 4  // LED que pisca a cada 200ms

static const char *TAG = "LED_Control";

// Tarefa para controlar o LED 1 (1000ms)
void led1_task(void *pvParameter) {
    // Configurar o pino do LED 1 como saída
    gpio_pad_select_gpio(LED_1);
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "LED 1 configurado no pino %d", LED_1);
    
    while(1) {
        // Acender o LED
        gpio_set_level(LED_1, 1);
        ESP_LOGI(TAG, "LED 1 ligado");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay de 1000ms
        
        // Apagar o LED
        gpio_set_level(LED_1, 0);
        ESP_LOGI(TAG, "LED 1 desligado");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay de 1000ms
    }
}

// Tarefa para controlar o LED 2 (200ms)
void led2_task(void *pvParameter) {
    // Configurar o pino do LED 2 como saída
    gpio_pad_select_gpio(LED_2);
    gpio_set_direction(LED_2, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "LED 2 configurado no pino %d", LED_2);
    
    while(1) {
        // Acender o LED
        gpio_set_level(LED_2, 1);
        ESP_LOGI(TAG, "LED 2 ligado");
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Delay de 200ms
        
        // Apagar o LED
        gpio_set_level(LED_2, 0);
        ESP_LOGI(TAG, "LED 2 desligado");
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Delay de 200ms
    }
}

void app_main() {
    // Criar as tarefas para controlar os LEDs
    xTaskCreate(&led1_task, "led1_task", 2048, NULL, 5, NULL);
    xTaskCreate(&led2_task, "led2_task", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Tarefas de controle dos LEDs iniciadas");
}
