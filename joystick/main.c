#include <stdio.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define JOYSTICK_X_ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36) para o eixo X
#define SERVO_PWM_GPIO GPIO_NUM_2              // Pino do servomotor (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0                // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE          // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0            // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT        // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 50                      // Frequência do PWM (50 Hz)

// Variáveis globais
static const char *TAG = "SERVO_JOYSTICK";

// Função para inicializar o ADC
void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura a resolução do ADC para 12 bits
    adc1_config_channel_atten(JOYSTICK_X_ADC_CHANNEL, ADC_ATTEN_DB_11); // Configura a atenuação para 11 dB
}

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
        .gpio_num = SERVO_PWM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // Duty cycle inicial (0%)
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

// Função para mapear o valor do ADC para o ângulo do servomotor
uint32_t map_adc_to_servo_angle(int adc_value) {
    // Mapeia o valor do ADC (0 a 4095) para o ângulo do servomotor (0° a 180°)
    return (adc_value * 180) / 4095;
}

// Função para ajustar o ângulo do servomotor
void set_servo_angle(uint32_t angle) {
    // Calcula o duty cycle correspondente ao ângulo
    // Para 50 Hz (20 ms), o duty cycle deve variar entre 1 ms (5%) e 2 ms (10%).
    // Para 13 bits (8192 valores), 5% = 409, 10% = 819.
    uint32_t duty = 409 + (angle * 410) / 180;

    // Define o duty cycle do PWM
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Atualiza o duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

// Função principal
void app_main() {
    // Inicializa o ADC e o PWM
    adc_init();
    pwm_init();

    while (1) {
        // Lê o valor do eixo X do joystick (0 a 4095)
        int joystick_x_value = adc1_get_raw(JOYSTICK_X_ADC_CHANNEL);

        // Mapeia o valor do eixo X para o ângulo do servomotor (0° a 180°)
        uint32_t servo_angle = map_adc_to_servo_angle(joystick_x_value);

        // Ajusta o ângulo do servomotor
        set_servo_angle(servo_angle);

        // Exibe os valores no terminal serial
        printf("Valor do eixo X: %d, Ângulo do servomotor: %" PRIu32 "\n", joystick_x_value, servo_angle);

        // Aguarda 100 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
