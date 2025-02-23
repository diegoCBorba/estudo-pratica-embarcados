#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Definições dos pinos
#define LED_GPIO GPIO_NUM_2  // Pino do LED
#define UART_TX_PIN GPIO_NUM_1  // Pino TX da UART
#define UART_RX_PIN GPIO_NUM_3  // Pino RX da UART
#define BUF_SIZE 1024  // Tamanho do buffer de recepção

static const char *TAG = "UART_INTERACTIVE";

// Função para configurar a UART
void uart_config() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configura a UART
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0));
}

// Função para configurar o LED
void led_config() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);  // Inicia com o LED desligado
}

// Função para exibir o menu de ajuda
void show_menu() {
    printf("\n--- Menu de Comandos ---\n");
    printf("led on       : Liga o LED\n");
    printf("led off      : Desliga o LED\n");
    printf("sensor       : Exibe o valor do sensor\n");
    printf("help         : Exibe este menu\n");
    printf("-----------------------\n");
}

// Função para simular a leitura de um sensor
int read_sensor() {
    // Simula a leitura de um sensor (valor aleatório entre 0 e 100)
    return esp_random() % 101;
}

// Função principal
void app_main() {
    char command[BUF_SIZE];  // Buffer para armazenar o comando recebido
    int len;  // Tamanho do comando recebido

    // Configura a UART e o LED
    uart_config();
    led_config();

    // Exibe o menu inicial
    show_menu();

    while (1) {
        // Lê o comando do terminal serial
        len = uart_read_bytes(UART_NUM_0, (uint8_t*)command, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            command[len] = '\0';  // Adiciona o terminador de string
            printf("Comando recebido: %s\n", command);

            // Processa o comando
            if (strcmp(command, "led on\n") == 0) {
                gpio_set_level(LED_GPIO, 1);  // Liga o LED
                printf("LED ligado.\n");
            } else if (strcmp(command, "led off\n") == 0) {
                gpio_set_level(LED_GPIO, 0);  // Desliga o LED
                printf("LED desligado.\n");
            } else if (strcmp(command, "sensor\n") == 0) {
                int sensor_value = read_sensor();  // Lê o valor do sensor
                printf("Valor do sensor: %d\n", sensor_value);
            } else if (strcmp(command, "help\n") == 0) {
                show_menu();  // Exibe o menu de ajuda
            } else {
                printf("Comando inválido. Digite 'help' para ver os comandos disponíveis.\n");
            }
        }
    }
}