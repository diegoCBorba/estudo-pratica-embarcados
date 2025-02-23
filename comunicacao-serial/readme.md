### [link wokwi](https://wokwi.com/projects/423714284044018689)

### **Estrutura Geral do Código**

O código implementa um **terminal serial interativo** para o ESP32, onde você pode enviar comandos via UART para controlar um LED e simular a leitura de um sensor. Ele usa a UART para comunicação serial e GPIO para controlar o LED.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`string.h`**: Biblioteca para manipulação de strings, como `strcmp`.
- **`driver/uart.h`**: Biblioteca do ESP-IDF para configurar e usar a UART.
- **`driver/gpio.h`**: Biblioteca do ESP-IDF para configurar e controlar GPIOs.
- **`esp_log.h`**: Biblioteca para logging (não usada diretamente neste código, mas comum em projetos ESP-IDF).
- **`esp_random.h`**: Biblioteca para gerar números aleatórios (usada para simular a leitura de um sensor).

---

### **Definições de Pinos e Constantes**

```c
#define LED_GPIO GPIO_NUM_2  // Pino do LED
#define UART_TX_PIN GPIO_NUM_1  // Pino TX da UART
#define UART_RX_PIN GPIO_NUM_3  // Pino RX da UART
#define BUF_SIZE 1024  // Tamanho do buffer de recepção
```

- **`LED_GPIO`**: Define o pino GPIO onde o LED está conectado (GPIO 2).
- **`UART_TX_PIN`**: Define o pino TX da UART (GPIO 1).
- **`UART_RX_PIN`**: Define o pino RX da UART (GPIO 3).
- **`BUF_SIZE`**: Define o tamanho do buffer para armazenar os comandos recebidos via UART.

---

### **Função `uart_config`**

```c
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
```

- **`uart_config_t`**: Estrutura que define a configuração da UART (baud rate, bits de dados, paridade, etc.).
- **`uart_param_config`**: Configura os parâmetros da UART.
- **`uart_set_pin`**: Define os pinos TX e RX da UART.
- **`uart_driver_install`**: Instala o driver da UART com um buffer de recepção de tamanho `BUF_SIZE * 2`.

---

### **Função `led_config`**

```c
void led_config() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);  // Inicia com o LED desligado
}
```

- **`gpio_reset_pin`**: Reseta o pino do LED para o estado padrão.
- **`gpio_set_direction`**: Configura o pino do LED como saída.
- **`gpio_set_level`**: Define o nível do pino (0 = desligado, 1 = ligado).

---

### **Função `show_menu`**

```c
void show_menu() {
    printf("\n--- Menu de Comandos ---\n");
    printf("led on       : Liga o LED\n");
    printf("led off      : Desliga o LED\n");
    printf("sensor       : Exibe o valor do sensor\n");
    printf("help         : Exibe este menu\n");
    printf("-----------------------\n");
}
```

- Exibe um menu de comandos disponíveis no terminal serial.

---

### **Função `read_sensor`**

```c
int read_sensor() {
    // Simula a leitura de um sensor (valor aleatório entre 0 e 100)
    return esp_random() % 101;
}
```

- **`esp_random`**: Gera um número aleatório.
- **`% 101`**: Limita o valor aleatório entre 0 e 100.

---

### **Função Principal `app_main`**

```c
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
```

- **`command[BUF_SIZE]`**: Buffer para armazenar o comando recebido.
- **`len`**: Tamanho do comando recebido.
- **`uart_read_bytes`**: Lê os dados recebidos via UART.
- **`strcmp`**: Compara strings para identificar o comando recebido.
- **`gpio_set_level`**: Liga ou desliga o LED conforme o comando.
- **`read_sensor`**: Simula a leitura de um sensor e exibe o valor.
- **`show_menu`**: Exibe o menu de ajuda.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura a UART e o LED.
   - Exibe o menu de comandos.

2. **Loop Principal**:
   - Lê comandos do terminal serial.
   - Processa os comandos:
     - `led on`: Liga o LED.
     - `led off`: Desliga o LED.
     - `sensor`: Exibe um valor aleatório do sensor.
     - `help`: Exibe o menu de ajuda.
   - Se o comando for inválido, exibe uma mensagem de erro.

---

### **Exemplo de Uso**

1. **Enviar Comandos**:
   - No terminal serial, digite:
     - `led on`: Liga o LED.
     - `led off`: Desliga o LED.
     - `sensor`: Exibe um valor aleatório do sensor.
     - `help`: Exibe o menu de ajuda.

2. **Saída Esperada**:
   ```
   --- Menu de Comandos ---
   led on       : Liga o LED
   led off      : Desliga o LED
   sensor       : Exibe o valor do sensor
   help         : Exibe este menu
   -----------------------

   Comando recebido: led on
   LED ligado.

   Comando recebido: sensor
   Valor do sensor: 42

   Comando recebido: led off
   LED desligado.
   ```

---

### **Conclusão**

Este código é um exemplo simples de como criar um terminal serial interativo para controlar um LED e simular a leitura de um sensor. Ele pode ser expandido para incluir mais funcionalidades, como controle de múltiplos LEDs, leitura de sensores reais, ou integração com outros protocolos de comunicação. 😊