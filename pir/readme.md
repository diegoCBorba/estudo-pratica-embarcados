### [link wokwi](https://wokwi.com/projects/423730529751705601)

### **Estrutura Geral do Código**

O código implementa um sistema de alarme de movimento usando um **sensor PIR** (Passive Infrared Sensor). Quando o sensor detecta movimento, ele ativa um **LED** e um **buzzer** como alerta. O status do sensor (movimento detectado ou não) é exibido no terminal serial.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`driver/gpio.h`**: Biblioteca do ESP-IDF para configurar e usar GPIOs.
- **`esp_log.h`**: Biblioteca para logging (exibição de mensagens no terminal serial).
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.

---

### **Definições e Constantes**

```c
#define PIR_SENSOR_GPIO GPIO_NUM_4  // Pino do sensor PIR (GPIO 4)
#define LED_GPIO GPIO_NUM_2         // Pino do LED (GPIO 2)
#define BUZZER_GPIO GPIO_NUM_5      // Pino do buzzer (GPIO 5)
```

- **`PIR_SENSOR_GPIO`**: Define o pino GPIO onde o sensor PIR está conectado (GPIO 4).
- **`LED_GPIO`**: Define o pino GPIO onde o LED está conectado (GPIO 2).
- **`BUZZER_GPIO`**: Define o pino GPIO onde o buzzer está conectado (GPIO 5).

---

### **Variáveis Globais**

```c
static const char *TAG = "PIR_ALARM";
```

- **`TAG`**: Tag usada para identificar as mensagens de log no terminal serial.

---

### **Função `gpio_init`**

```c
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
```

- **`gpio_reset_pin`**: Reseta o pino para o estado padrão.
- **`gpio_set_direction`**: Configura o pino como entrada ou saída.
  - **`GPIO_MODE_INPUT`**: Configura o pino como entrada (para o sensor PIR).
  - **`GPIO_MODE_OUTPUT`**: Configura o pino como saída (para o LED e o buzzer).

---

### **Função Principal `app_main`**

```c
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
```

1. **Inicialização**:
   - Configura os GPIOs usando a função `gpio_init`.

2. **Loop Principal**:
   - Lê o estado do sensor PIR usando `gpio_get_level`.
   - Se o movimento for detectado (estado alto), ativa o LED e o buzzer usando `gpio_set_level` e exibe uma mensagem de log.
   - Se nenhum movimento for detectado (estado baixo), desativa o LED e o buzzer e exibe uma mensagem de log.
   - Aguarda 500 ms antes de repetir o processo usando `vTaskDelay`.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura os GPIOs para o sensor PIR, LED e buzzer.

2. **Leitura do Sensor PIR**:
   - O estado do sensor PIR é lido a cada 500 ms.

3. **Ativação do Alarme**:
   - Se o movimento for detectado, o LED e o buzzer são ativados.
   - Se nenhum movimento for detectado, o LED e o buzzer são desativados.

4. **Exibição de Status**:
   - O status do sensor é exibido no terminal serial.

---

### **Exemplo de Saída**

```
I (500) PIR_ALARM: Nenhum movimento detectado.
I (1000) PIR_ALARM: Movimento detectado!
I (1500) PIR_ALARM: Nenhum movimento detectado.
```

---

### **Conclusão**

Este código é uma implementação simples e eficaz de um alarme de movimento usando um sensor PIR no ESP32. Ele pode ser facilmente expandido para incluir mais funcionalidades, como notificações remotas ou integração com outros dispositivos. Se precisar de mais detalhes ou ajuda, é só perguntar! 😊