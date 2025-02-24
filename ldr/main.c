#include <stdio.h>
#include <inttypes.h>  // Para usar PRIu32
#include "esp_adc/adc_oneshot.h"  // Nova API do ADC
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define LDR_ADC_CHANNEL ADC_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define LED_GPIO GPIO_NUM_2             // Pino do LED (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0         // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0     // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 5000             // Frequência do PWM (5 kHz)

// Variáveis globais
adc_oneshot_unit_handle_t adc_handle;  // Handle para o ADC

// Função para inicializar o ADC
void adc_init() {
    // Configuração do ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,  // Usar ADC1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configuração do canal do ADC
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,  // Atenuação de 12 dB (0 a 3.3V)
        .bitwidth = ADC_BITWIDTH_12,  // Resolução de 12 bits
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &channel_config));
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
    // Inicializa o ADC e o PWM
    adc_init();
    pwm_init();

    // Define o valor máximo do duty cycle (2^13 - 1 = 8191)
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;

    while (1) {
        // Lê o valor do LDR (0 a 4095)
        int ldr_value;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &ldr_value));

        // Mapeia o valor do LDR para o duty cycle do PWM (0 a 8191)
        uint32_t led_duty = ((ldr_value * max_duty) / 4095) - 64;

        // Ajusta o brilho do LED
        set_led_brightness(led_duty);

        // Exibe os valores no terminal serial
        printf("Valor do LDR: %d, Brilho do LED: %" PRIu32 "\n", ldr_value, led_duty);

        // Aguarda 100 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}