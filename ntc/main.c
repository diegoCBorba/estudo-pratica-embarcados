#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definições
#define NTC_ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define R_REF 10000.0                   // Resistência de referência (10 kΩ)
#define BETA 3950.0                     // Coeficiente Beta do NTC
#define T0 298.15                       // Temperatura de referência (25°C em Kelvin)
#define R0 10000.0                      // Resistência do NTC a 25°C (10 kΩ)

// Variáveis globais
static const char *TAG = "NTC_TEMP_MONITOR";

// Função para inicializar o ADC
void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura a resolução do ADC para 12 bits
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11); // Configura a atenuação para 11 dB
}

// Função para ler a resistência do NTC
float read_ntc_resistance() {
    // Lê o valor do ADC (0 a 4095)
    int adc_value = adc1_get_raw(NTC_ADC_CHANNEL);

    // Calcula a resistência do NTC
    float v_out = (adc_value / 4095.0) * 3.3; // Tensão no pino do NTC
    float r_ntc = (R_REF * v_out) / (3.3 - v_out); // Resistência do NTC

    return r_ntc;
}

// Função para converter a resistência em temperatura (em °C)
float convert_resistance_to_temperature(float r_ntc) {
    // Equação de Steinhart-Hart
    float steinhart = log(r_ntc / R0) / BETA; // ln(R/R0) / B
    steinhart += 1.0 / T0;                   // + 1/T0
    steinhart = 1.0 / steinhart;             // 1 / (ln(R/R0) / B + 1/T0)

    // Converte de Kelvin para Celsius
    float temperature = steinhart - 273.15;

    return temperature;
}

// Função principal
void app_main() {
    // Inicializa o ADC
    adc_init();

    while (1) {
        // Lê a resistência do NTC
        float r_ntc = read_ntc_resistance();

        // Converte a resistência em temperatura
        float temperature = convert_resistance_to_temperature(r_ntc);

        // Exibe os valores no terminal serial
        printf("Resistência do NTC: %.2f Ω, Temperatura: %.2f °C\n", r_ntc, temperature);

        // Aguarda 1 segundo antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
