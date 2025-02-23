### [link wokwi](https://wokwi.com/projects/423714284044018689)

### **Estrutura Geral do Código**

O código implementa a leitura de um valor analógico de um potenciômetro (ou outro sensor analógico) conectado a um pino ADC do ESP32. Ele aplica um **filtro de média móvel** para suavizar as leituras e exibe os valores brutos e suavizados no terminal serial.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`driver/adc.h`**: Biblioteca do ESP-IDF para configurar e usar o ADC.
- **`esp_log.h`**: Biblioteca para logging (não usada diretamente neste código, mas comum em projetos ESP-IDF).
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.

---

### **Definições e Variáveis Globais**

```c
#define ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define ADC_WIDTH ADC_WIDTH_BIT_12  // Resolução de 12 bits (0 a 4095)
#define ADC_ATTEN ADC_ATTEN_DB_11   // Atenuação de 11 dB (0 a 3.3V)
#define FILTER_SIZE 10              // Tamanho do filtro de média móvel

static const char *TAG = "ADC_SMOOTHING";
int adc_readings[FILTER_SIZE];      // Array para armazenar as leituras do ADC
int adc_index = 0;                  // Índice atual do array
```

- **`ADC_CHANNEL`**: Define o canal ADC a ser usado (ADC1_CHANNEL_0 corresponde ao GPIO 36).
- **`ADC_WIDTH`**: Define a resolução do ADC como 12 bits (valores de 0 a 4095).
- **`ADC_ATTEN`**: Define a atenuação do ADC como 11 dB, permitindo leituras de 0 a 3.3V.
- **`FILTER_SIZE`**: Define o tamanho do filtro de média móvel (10 leituras).
- **`TAG`**: Tag para logging (não usada neste código).
- **`adc_readings`**: Array para armazenar as últimas leituras do ADC.
- **`adc_index`**: Índice para controlar a posição atual no array de leituras.

---

### **Função `adc_init`**

```c
void adc_init() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}
```

- **`adc1_config_width`**: Configura a resolução do ADC (12 bits).
- **`adc1_config_channel_atten`**: Configura a atenuação do canal ADC (11 dB).

---

### **Função `read_adc`**

```c
int read_adc() {
    return adc1_get_raw(ADC_CHANNEL);
}
```

- **`adc1_get_raw`**: Lê o valor bruto do ADC no canal especificado (ADC1_CHANNEL_0).

---

### **Função `apply_moving_average`**

```c
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
```

- **`adc_readings[adc_index] = new_value`**: Armazena o novo valor no array de leituras.
- **`adc_index = (adc_index + 1) % FILTER_SIZE`**: Atualiza o índice para a próxima posição no array (usando módulo para voltar ao início após o fim).
- **`sum += adc_readings[i]`**: Soma todos os valores no array.
- **`return sum / FILTER_SIZE`**: Retorna a média dos valores no array.

---

### **Função Principal `app_main`**

```c
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
```

- **`adc_init`**: Inicializa o ADC.
- **`for (int i = 0; i < FILTER_SIZE; i++)`**: Inicializa o array de leituras com zeros.
- **`while (1)`**: Loop infinito que lê o valor do ADC, aplica o filtro de média móvel e exibe os valores no terminal serial.
  - **`read_adc`**: Lê o valor bruto do ADC.
  - **`apply_moving_average`**: Aplica o filtro de média móvel.
  - **`printf`**: Exibe os valores brutos e suavizados no terminal serial.
  - **`vTaskDelay`**: Aguarda 100 ms antes da próxima leitura.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura o ADC e inicializa o array de leituras.

2. **Loop Principal**:
   - Lê o valor bruto do ADC.
   - Aplica o filtro de média móvel para suavizar as leituras.
   - Exibe os valores brutos e suavizados no terminal serial.
   - Aguarda 100 ms antes de repetir o processo.

---

### **Exemplo de Saída**

```
Valor bruto: 2048, Valor suavizado: 2050
Valor bruto: 2055, Valor suavizado: 2052
Valor bruto: 2060, Valor suavizado: 2055
Valor bruto: 2058, Valor suavizado: 2056
```

---

### **Conclusão**

Este código é uma implementação simples e eficaz de um filtro de média móvel para suavizar leituras analógicas no ESP32. Ele pode ser facilmente adaptado para diferentes sensores e aplicações. Se precisar de mais detalhes ou ajuda, é só perguntar! 😊