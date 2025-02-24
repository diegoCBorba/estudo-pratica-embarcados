### [link wokwi](https://wokwi.com/projects/423735265167723521)

### **Estrutura Geral do Código**

O código implementa um sistema que lê valores de um **sensor analógico** através do **ADC** (Conversor Analógico-Digital) do ESP32 e os utiliza para controlar um **LED Bar Graph**. O valor do ADC determina quantos LEDs serão acesos proporcionalmente ao seu valor, criando uma representação visual da leitura do sensor.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
```

- **`stdio.h`**: Biblioteca padrão para entrada e saída, usada para `printf`.
- **`freertos/FreeRTOS.h`** e **`freertos/task.h`**: Bibliotecas do FreeRTOS, utilizadas para controle de tarefas e delays.
- **`esp_adc/adc_oneshot.h`**: Biblioteca do ESP-IDF para leitura do ADC usando o novo driver `adc_oneshot`.
- **`driver/gpio.h`**: Biblioteca para controle dos pinos GPIO do ESP32.

---

### **Definições e Constantes**

```c
#define ADC_CHANNEL ADC_CHANNEL_6  // GPIO34
#define ADC_BITWIDTH ADC_BITWIDTH_12  // Resolução de 12 bits
#define ADC_ATTEN ADC_ATTEN_DB_12  // Atenuação de 12 dB

#define LED_COUNT 10
const int led_pins[LED_COUNT] = {2, 4, 5, 18, 19, 21, 22, 23, 25, 26};
```

- **`ADC_CHANNEL`**: Define o canal ADC que será utilizado (ADC_CHANNEL_6 corresponde ao GPIO34).
- **`ADC_BITWIDTH`**: Define a resolução do ADC como **12 bits** (valores entre 0 e 4095).
- **`ADC_ATTEN`**: Define a atenuação do ADC para **12 dB**, permitindo leituras até aproximadamente 3.9V.
- **`LED_COUNT`**: Define a quantidade de LEDs no bar graph.
- **`led_pins[]`**: Vetor contendo os **pinos GPIO** dos LEDs.

---

### **Variáveis Globais**

```c
adc_oneshot_unit_handle_t adc1_handle;
```

- **`adc1_handle`**: Estrutura para gerenciar o ADC no ESP32 utilizando o driver **adc_oneshot**.

---

### **Função `configure_adc`**

```c
void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,  
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
}
```

1. **Inicializa o ADC**: Cria uma nova instância de `adc_oneshot` para gerenciar leituras.
2. **Configura o canal ADC**:
   - Define a **resolução** como 12 bits.
   - Define a **atenuação** como 12 dB.

---

### **Função `configure_leds`**

```c
void configure_leds() {
    for (int i = 0; i < LED_COUNT; i++) {
        esp_rom_gpio_pad_select_gpio(led_pins[i]);  
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
    }
}
```

1. **Configura os pinos dos LEDs como saída**.
2. **Utiliza `esp_rom_gpio_pad_select_gpio`** para selecionar os pinos corretamente.

---

### **Função `update_led_bar`**

```c
void update_led_bar(int value) {
    int level = (value * LED_COUNT) / 4095;
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_set_level(led_pins[i], i < level ? 1 : 0);
    }
}
```

1. **Calcula quantos LEDs devem acender** com base na leitura do ADC.
2. **Ativa os LEDs proporcionalmente** ao valor lido.

---

### **Função Principal `app_main`**

```c
void app_main() {
    configure_adc();
    configure_leds();

    while (1) {
        int adc_value = 0;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_value);
        printf("ADC Value: %d\n", adc_value);
        update_led_bar(adc_value);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

1. **Configura o ADC e os LEDs**.
2. **Lê o valor do ADC** e exibe no terminal.
3. **Atualiza o LED Bar Graph** com base no valor lido.
4. **Repete a leitura a cada 200ms**.

---

### **Fluxo de Funcionamento**

1. **Leitura do Sensor**:
   - O ESP32 lê valores do ADC no **GPIO34** (ADC_CHANNEL_6).
   
2. **Processamento dos Dados**:
   - O valor do ADC (0 a 4095) é convertido para um número de LEDs a serem acesos.

3. **Exibição no LED Bar Graph**:
   - LEDs acendem de acordo com a intensidade da leitura.

4. **Loop Contínuo**:
   - O processo se repete a cada **200ms**.

---

### **Exemplo de Saída**

```
ADC Value: 1023
ADC Value: 2050
ADC Value: 3075
```

---

### **Conclusão**

Este código permite visualizar **valores analógicos** em um **LED Bar Graph** utilizando o ADC do ESP32. Ele pode ser facilmente adaptado para monitoramento de sensores como **LDR, potenciômetros e sensores de temperatura**.