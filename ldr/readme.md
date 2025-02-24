### [link wokwi](https://wokwi.com/projects/423728575317981185)

### **Estrutura Geral do Código**

O código implementa um sistema que lê a intensidade da luz usando um **sensor LDR** (Light Dependent Resistor) e ajusta o brilho de um **LED** proporcionalmente à luminosidade detectada. O ESP32 usa o **ADC** (Analog-to-Digital Converter) para ler o valor do LDR e o **PWM** (Pulse Width Modulation) para controlar o brilho do LED.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include <inttypes.h>  // Para usar PRIu32
#include "esp_adc/adc_oneshot.h"  // Nova API do ADC
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`inttypes.h`**: Biblioteca para usar macros como `PRIu32`, que ajudam a formatar valores inteiros de maneira portável.
- **`esp_adc/adc_oneshot.h`**: Biblioteca do ESP-IDF para configurar e usar o ADC em modo de leitura única.
- **`driver/ledc.h`**: Biblioteca do ESP-IDF para configurar e usar o PWM.
- **`esp_err.h`**: Biblioteca para manipulação de erros no ESP-IDF.
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.

---

### **Definições e Constantes**

```c
#define LDR_ADC_CHANNEL ADC_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define LED_GPIO GPIO_NUM_2             // Pino do LED (GPIO 2)
#define LEDC_TIMER LEDC_TIMER_0         // Timer do PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // Modo de velocidade do PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0     // Canal do PWM
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do duty cycle (13 bits)
#define LEDC_FREQUENCY 5000             // Frequência do PWM (5 kHz)
```

- **`LDR_ADC_CHANNEL`**: Define o canal ADC a ser usado (ADC1_CHANNEL_0 corresponde ao GPIO 36).
- **`LED_GPIO`**: Define o pino GPIO onde o LED está conectado (GPIO 2).
- **`LEDC_TIMER`**: Define o timer do PWM (LEDC_TIMER_0).
- **`LEDC_MODE`**: Define o modo de velocidade do PWM (LEDC_LOW_SPEED_MODE).
- **`LEDC_CHANNEL`**: Define o canal do PWM (LEDC_CHANNEL_0).
- **`LEDC_DUTY_RES`**: Define a resolução do duty cycle como 13 bits (valores de 0 a 8191).
- **`LEDC_FREQUENCY`**: Define a frequência do PWM como 5 kHz.

---

### **Variáveis Globais**

```c
adc_oneshot_unit_handle_t adc_handle;  // Handle para o ADC
```

- **`adc_handle`**: Variável que armazena o handle (identificador) do ADC, usado para realizar leituras.

---

### **Função `adc_init`**

```c
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
```

- **`adc_oneshot_unit_init_cfg_t`**: Estrutura que define a configuração inicial do ADC.
  - **`unit_id`**: Define a unidade ADC a ser usada (ADC_UNIT_1 para ADC1).
- **`adc_oneshot_new_unit`**: Inicializa o ADC com a configuração fornecida e retorna um handle.
- **`adc_oneshot_chan_cfg_t`**: Estrutura que define a configuração do canal do ADC.
  - **`atten`**: Define a atenuação do sinal (ADC_ATTEN_DB_12 para 0 a 3.3V).
  - **`bitwidth`**: Define a resolução do ADC (ADC_BITWIDTH_12 para 12 bits).
- **`adc_oneshot_config_channel`**: Configura o canal do ADC com a configuração fornecida.

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
- **`ledc_timer_config`**: Configura o timer do PWM.
- **`ledc_channel_config_t`**: Estrutura que define a configuração do canal do PWM.
  - **`gpio_num`**: Pino GPIO onde o LED está conectado (GPIO 2).
  - **`speed_mode`**: Modo de velocidade (LEDC_LOW_SPEED_MODE).
  - **`channel`**: Canal do PWM (LEDC_CHANNEL_0).
  - **`timer_sel`**: Timer associado ao canal (LEDC_TIMER_0).
  - **`duty`**: Duty cycle inicial (0%).
  - **`hpoint`**: Ponto de referência para o duty cycle (0).
- **`ledc_channel_config`**: Configura o canal do PWM.

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
```

1. **Inicialização**:
   - Configura o ADC e o PWM.

2. **Loop Principal**:
   - Lê o valor do LDR usando `adc_oneshot_read`.
   - Mapeia o valor do LDR (0 a 4095) para o duty cycle do PWM (0 a 8191).
   - Ajusta o brilho do LED usando `set_led_brightness`.
   - Exibe os valores lidos e o brilho do LED no terminal serial.
   - Aguarda 100 ms antes de repetir o processo.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura o ADC e o PWM.

2. **Leitura do LDR**:
   - O valor do LDR é lido e mapeado para o duty cycle do PWM.

3. **Controle do LED**:
   - O brilho do LED é ajustado proporcionalmente ao valor lido do LDR.

4. **Exibição dos Valores**:
   - Os valores lidos e o brilho do LED são exibidos no terminal serial.

5. **Repetição**:
   - O processo é repetido a cada 100 ms.

---

### **Exemplo de Saída**

```
Valor do LDR: 1024, Brilho do LED: 2048
Valor do LDR: 2048, Brilho do LED: 4096
Valor do LDR: 3072, Brilho do LED: 6144
```

---

### **Conclusão**

Este código é uma implementação simples e eficaz de um sistema que ajusta o brilho de um LED proporcionalmente à luminosidade detectada por um sensor LDR. Ele pode ser facilmente adaptado para diferentes sensores e aplicações. Se precisar de mais detalhes ou ajuda, é só perguntar! 😊