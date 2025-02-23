#include <stdio.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define ADC_WIDTH ADC_WIDTH_BIT_12  // Resolução de 12 bits (0 a 4095)
#define ADC_ATTEN ADC_ATTEN_DB_11   // Atenuação de 11 dB (0 a 3.3V)
#define FILTER_SIZE 10              // Tamanho do filtro de média móvel

// Variáveis globais
static const char *TAG = "ADC_SMOOTHING";
int adc_readings[FILTER_SIZE];      // Array para armazenar as leituras do ADC
int adc_index = 0;                  // Índice atual do array

// Função para inicializar o ADC
void adc_init() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

// Função para ler o valor do ADC
int read_adc() {
    return adc1_get_raw(ADC_CHANNEL);
}

// Função para aplicar o filtro de média móvel
int apply_moving_average(int new_value) {
    // Adiciona o novo valor ao array
    adc_readings[adc_index] = new_value;
    adc_index = (adc_index + 1) % FILTER_SIZE;

    // Calcula a média dos valores no array
    int sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += adc_readings[i];
    }
    return sum / FILTER_SIZE;
}

// Função principal
void app_main() {
    // Inicializa o ADC
    adc_init();

    // Inicializa o array de leituras com zeros
    for (int i = 0; i < FILTER_SIZE; i++) {
        adc_readings[i] = 0;
    }

    while (1) {
        // Lê o valor bruto do ADC
        int raw_value = read_adc();

        // Aplica o filtro de média móvel
        int smoothed_value = apply_moving_average(raw_value);

        // Exibe os valores no terminal serial
        printf("Valor bruto: %d, Valor suavizado: %d\n", raw_value, smoothed_value);

        // Aguarda 100 ms antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
