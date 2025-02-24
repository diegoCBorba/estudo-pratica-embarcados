#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define LED_GPIO GPIO_NUM_2          // Pino do LED (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0      // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0  // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 5000          // Frequência do PWM (5 kHz)

// Função para inicializar o PWM
void pwm_init() {
    // Configura o timer do PWM
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Configura o canal do PWM
    ledc_channel_config_t channel_config = {
        .gpio_num = LED_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // Duty cycle inicial (0%)
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

// Função para ajustar o brilho do LED
void set_led_brightness(uint32_t duty) {
    // Define o duty cycle do PWM
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Atualiza o duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

// Função principal
void app_main() {
    // Inicializa o PWM
    pwm_init();

    // Define o valor máximo do duty cycle (2^13 - 1 = 8191)
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;

    while (1) {
        // Aumenta o brilho do LED
        for (int duty = 0; duty < max_duty; duty += 100) {
            set_led_brightness(duty);
            vTaskDelay(pdMS_TO_TICKS(10)); // Aguarda 10 ms
        }

        // Diminui o brilho do LED
        for (int duty = max_duty; duty > 0; duty -= 100) {
            set_led_brightness(duty);
            vTaskDelay(pdMS_TO_TICKS(10)); // Aguarda 10 ms
        }
    }
}