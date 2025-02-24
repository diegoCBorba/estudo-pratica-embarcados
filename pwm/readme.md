### [link wokwi](https://wokwi.com/projects/423728575317981185)

### **Estrutura Geral do Código**

O código implementa o controle do brilho de um LED usando **PWM** (Pulse Width Modulation) no ESP32. O PWM é uma técnica que varia a largura do pulso de um sinal digital para simular um sinal analógico, permitindo controlar o brilho de um LED ou a velocidade de um motor.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`driver/ledc.h`**: Biblioteca do ESP-IDF para configurar e usar o PWM.
- **`esp_err.h`**: Biblioteca para manipulação de erros no ESP-IDF.
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.

---

### **Definições e Constantes**

```c
#define LED_GPIO GPIO_NUM_2          // Pino do LED (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0      // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0  // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 5000          // Frequência do PWM (5 kHz)
```

- **`LED_GPIO`**: Define o pino GPIO onde o LED está conectado (GPIO 2).
- **`LEDC_TIMER`**: Define o timer do PWM (LEDC_TIMER_0).
- **`LEDC_MODE`**: Define o modo de velocidade do PWM (LEDC_LOW_SPEED_MODE).
- **`LEDC_CHANNEL`**: Define o canal do PWM (LEDC_CHANNEL_0).
- **`LEDC_DUTY_RES`**: Define a resolução do duty cycle como 13 bits (valores de 0 a 8191).
- **`LEDC_FREQUENCY`**: Define a frequência do PWM como 5 kHz.

---

### **Função `pwm_init`**

```c
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
```

- **`ledc_timer_config_t`**: Estrutura que define a configuração do timer do PWM.
  - **`speed_mode`**: Modo de velocidade (LEDC_LOW_SPEED_MODE).
  - **`duty_resolution`**: Resolução do duty cycle (13 bits).
  - **`timer_num`**: Timer do PWM (LEDC_TIMER_0).
  - **`freq_hz`**: Frequência do PWM (5 kHz).
  - **`clk_cfg`**: Configuração do clock (LEDC_AUTO_CLK).

- **`ledc_channel_config_t`**: Estrutura que define a configuração do canal do PWM.
  - **`gpio_num`**: Pino GPIO onde o LED está conectado (GPIO 2).
  - **`speed_mode`**: Modo de velocidade (LEDC_LOW_SPEED_MODE).
  - **`channel`**: Canal do PWM (LEDC_CHANNEL_0).
  - **`timer_sel`**: Timer associado ao canal (LEDC_TIMER_0).
  - **`duty`**: Duty cycle inicial (0%).
  - **`hpoint`**: Ponto de referência para o duty cycle (0).

- **`ESP_ERROR_CHECK`**: Verifica se a configuração foi bem-sucedida e trata erros.

---

### **Função `set_led_brightness`**

```c
void set_led_brightness(uint32_t duty) {
    // Define o duty cycle do PWM
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Atualiza o duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}
```

- **`ledc_set_duty`**: Define o duty cycle do PWM (valor entre 0 e 8191).
- **`ledc_update_duty`**: Atualiza o duty cycle no hardware.

---

### **Função Principal `app_main`**

```c
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
```

- **`pwm_init`**: Inicializa o PWM.
- **`max_duty`**: Define o valor máximo do duty cycle (8191 para 13 bits).
- **`while (1)`**: Loop infinito que aumenta e diminui o brilho do LED.
  - **`for (int duty = 0; duty < max_duty; duty += 100)`**: Aumenta o brilho do LED gradualmente.
  - **`for (int duty = max_duty; duty > 0; duty -= 100)`**: Diminui o brilho do LED gradualmente.
  - **`vTaskDelay(pdMS_TO_TICKS(10))`**: Aguarda 10 ms antes da próxima iteração.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura o timer e o canal do PWM.

2. **Loop Principal**:
   - Aumenta o brilho do LED de 0 a 8191 em incrementos de 100.
   - Diminui o brilho do LED de 8191 a 0 em decrementos de 100.
   - O processo é repetido continuamente.

---

### **Exemplo de Saída**

- O brilho do LED varia suavemente de desligado (0) até o brilho máximo (8191) e vice-versa.

---

### **Conclusão**

Este código é uma implementação simples e eficaz de controle de brilho de LED usando PWM no ESP32. Ele pode ser facilmente adaptado para controlar a velocidade de um motor ou outros dispositivos que utilizam PWM. Se precisar de mais detalhes ou ajuda, é só perguntar! 😊