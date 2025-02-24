### [link wokwi](https://wokwi.com/projects/423732449542627329)

### **Estrutura Geral do Código**

O código implementa um sistema para controlar um **servomotor** usando um **joystick**. Ele lê o valor do eixo X do joystick através do **ADC** (Analog-to-Digital Converter) do ESP32 e mapeia esse valor para um ângulo de 0° a 180°. O ângulo do servomotor é ajustado via **PWM** (Pulse Width Modulation), e o valor do joystick e o ângulo do servomotor são exibidos no terminal serial.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`driver/adc.h`**: Biblioteca do ESP-IDF para configurar e usar o ADC.
- **`driver/ledc.h`**: Biblioteca do ESP-IDF para controlar o PWM, usado para o controle do servomotor.
- **`esp_log.h`**: Biblioteca para logging (exibição de mensagens no terminal serial).
- **`freertos/FreeRTOS.h`** e **`freertos/task.h`**: Bibliotecas do FreeRTOS, que gerenciam tarefas e delays.

---

### **Definições e Constantes**

```c
#define JOYSTICK_X_ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define SERVO_PWM_GPIO GPIO_NUM_2              // Pino do servomotor (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0                // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE          // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0            // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT        // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 50                      // Frequência do PWM (50 Hz)
```

- **`JOYSTICK_X_ADC_CHANNEL`**: Define o canal ADC a ser usado (ADC1_CHANNEL_0 corresponde ao GPIO 36).
- **`SERVO_PWM_GPIO`**: Define o pino GPIO (GPIO 2) que controla o servomotor.
- **`LEDC_TIMER`, `LEDC_MODE`, `LEDC_CHANNEL`, `LEDC_DUTY_RES`, `LEDC_FREQUENCY`**: Configurações do PWM, como o timer, modo de velocidade, canal, resolução do duty cycle e a frequência de 50 Hz.

---

### **Variáveis Globais**

```c
static const char *TAG = "SERVO_JOYSTICK";
```

- **`TAG`**: Tag usada para identificar as mensagens de log no terminal serial.

---

### **Função `adc_init`**

```c
void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura a resolução do ADC para 12 bits
    adc1_config_channel_atten(JOYSTICK_X_ADC_CHANNEL, ADC_ATTEN_DB_11); // Configura a atenuação para 11 dB
}
```

- **`adc1_config_width`**: Configura a resolução do ADC para 12 bits, permitindo valores de 0 a 4095.
- **`adc1_config_channel_atten`**: Configura a atenuação do canal ADC para 11 dB, permitindo leituras de 0 a 3.3V.

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
        .gpio_num = SERVO_PWM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // Duty cycle inicial (0%)
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}
```

- **`ledc_timer_config`**: Configura o timer do PWM, definindo a resolução e a frequência de 50 Hz.
- **`ledc_channel_config`**: Configura o canal de PWM para controlar o servomotor, com um duty cycle inicial de 0% (posição inicial).

---

### **Função `map_adc_to_servo_angle`**

```c
uint32_t map_adc_to_servo_angle(int adc_value) {
    // Mapeia o valor do ADC (0 a 4095) para o ângulo do servomotor (0° a 180°)
    return (adc_value * 180) / 4095;
}
```

- **Função de Mapeamento**: Recebe o valor do ADC (0 a 4095) e mapeia esse valor para um ângulo de 0° a 180°. Assim, o movimento do joystick no eixo X controla o ângulo do servomotor.

---

### **Função `set_servo_angle`**

```c
void set_servo_angle(uint32_t angle) {
    // Calcula o duty cycle correspondente ao ângulo
    uint32_t duty = 409 + (angle * 410) / 180;

    // Define o duty cycle do PWM
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Atualiza o duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}
```

1. **Cálculo do Duty Cycle**:
   - A função calcula o duty cycle necessário para mover o servomotor entre 0° e 180°, onde 5% de duty (1 ms) move o servomotor para 0° e 10% de duty (2 ms) move-o para 180°.
   
2. **Definição e Atualização do Duty Cycle**:
   - Define e atualiza o duty cycle no canal de PWM para ajustar a posição do servomotor.

---

### **Função Principal `app_main`**

```c
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
```

1. **Inicialização**:
   - Inicializa o ADC e o PWM para controlar o joystick e o servomotor.

2. **Loop Principal**:
   - Lê o valor do eixo X do joystick, mapeia esse valor para um ângulo de servomotor e ajusta a posição do servomotor.
   - Exibe os valores lidos e ajustados no terminal serial.
   - Aguarda 100 ms antes de realizar a próxima leitura.

---

### **Fluxo de Funcionamento**

1. **Leitura do Joystick**:
   - O código lê continuamente o valor do eixo X do joystick através do ADC.

2. **Mapeamento e Ajuste do Servo**:
   - O valor do ADC é mapeado para um ângulo de 0° a 180° e o servomotor é ajustado de acordo com o valor.

3. **Exibição no Terminal**:
   - O valor lido do joystick e o ângulo calculado do servomotor são exibidos no terminal serial.

4. **Repetição**:
   - O processo é repetido a cada 100 ms.

---

### **Exemplo de Saída**

```
Valor do eixo X: 2048, Ângulo do servomotor: 90
Valor do eixo X: 1024, Ângulo do servomotor: 45
Valor do eixo X: 3072, Ângulo do servomotor: 135
```

---

### **Conclusão**

Este código permite controlar a posição de um servomotor usando um joystick com a ajuda de um ADC e PWM no ESP32. Ele pode ser facilmente adaptado para diferentes aplicações que envolvem controle de movimento ou dispositivos interativos. Se precisar de mais detalhes ou ajuda, estou à disposição! 😊