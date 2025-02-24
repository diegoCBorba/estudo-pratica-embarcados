#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define PIR_SENSOR_GPIO GPIO_NUM_4  // Pino do sensor PIR (GPIO 4)
#define LED_GPIO GPIO_NUM_2         // Pino do LED (GPIO 2)
#define BUZZER_GPIO GPIO_NUM_5      // Pino do buzzer (GPIO 5)

// Variáveis globais
static const char *TAG = "PIR_ALARM";

// Função para inicializar os GPIOs
void gpio_init() {
    // Configura o pino do sensor PIR como entrada
    gpio_reset_pin(PIR_SENSOR_GPIO);
    gpio_set_direction(PIR_SENSOR_GPIO, GPIO_MODE_INPUT);

    // Configura o pino do LED como saída
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Configura o pino do buzzer como saída
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
}

// Função principal
void app_main() {
    // Inicializa os GPIOs
    gpio_init();

    while (1) {
        // Lê o estado do sensor PIR
        int pir_state = gpio_get_level(PIR_SENSOR_GPIO);

        // Verifica se o movimento foi detectado
        if (pir_state == 1) {
            // Ativa o LED e o buzzer
            gpio_set_level(LED_GPIO, 1);
            gpio_set_level(BUZZER_GPIO, 1);
            ESP_LOGI(TAG, "Movimento detectado!");
        } else {
            // Desativa o LED e o buzzer
            gpio_set_level(LED_GPIO, 0);
            gpio_set_level(BUZZER_GPIO, 0);
            ESP_LOGI(TAG, "Nenhum movimento detectado.");
        }

        // Aguarda 500 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}