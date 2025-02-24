### [link wokwi](https://wokwi.com/projects/423731055753159681)

### **Estrutura Geral do Código**

O código implementa um sistema para monitorar a temperatura usando um **sensor NTC** (Negative Temperature Coefficient). Ele lê a resistência do NTC usando o **ADC** (Analog-to-Digital Converter) do ESP32 e converte o valor lido em temperatura usando a equação de Steinhart-Hart. A resistência e a temperatura são exibidas no terminal serial.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`math.h`**: Biblioteca para funções matemáticas, como `log`.
- **`driver/adc.h`**: Biblioteca do ESP-IDF para configurar e usar o ADC.
- **`esp_log.h`**: Biblioteca para logging (exibição de mensagens no terminal serial).
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.

---

### **Definições e Constantes**

```c
#define NTC_ADC_CHANNEL ADC1_CHANNEL_0  // Pino ADC1_0 (GPIO 36)
#define R_REF 10000.0                   // Resistência de referência (10 kΩ)
#define BETA 3950.0                     // Coeficiente Beta do NTC
#define T0 298.15                       // Temperatura de referência (25°C em Kelvin)
#define R0 10000.0                      // Resistência do NTC a 25°C (10 kΩ)
```

- **`NTC_ADC_CHANNEL`**: Define o canal ADC a ser usado (ADC1_CHANNEL_0 corresponde ao GPIO 36).
- **`R_REF`**: Resistência de referência (10 kΩ).
- **`BETA`**: Coeficiente Beta do NTC (3950).
- **`T0`**: Temperatura de referência em Kelvin (25°C = 298.15 K).
- **`R0`**: Resistência do NTC a 25°C (10 kΩ).

---

### **Variáveis Globais**

```c
static const char *TAG = "NTC_TEMP_MONITOR";
```

- **`TAG`**: Tag usada para identificar as mensagens de log no terminal serial.

---

### **Função `adc_init`**

```c
void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura a resolução do ADC para 12 bits
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11); // Configura a atenuação para 11 dB
}
```

- **`adc1_config_width`**: Configura a resolução do ADC para 12 bits (valores de 0 a 4095).
- **`adc1_config_channel_atten`**: Configura a atenuação do canal ADC para 11 dB (permite leituras de 0 a 3.3V).

---

### **Função `read_ntc_resistance`**

```c
float read_ntc_resistance() {
    // Lê o valor do ADC (0 a 4095)
    int adc_value = adc1_get_raw(NTC_ADC_CHANNEL);

    // Calcula a resistência do NTC
    float v_out = (adc_value / 4095.0) * 3.3; // Tensão no pino do NTC
    float r_ntc = (R_REF * v_out) / (3.3 - v_out); // Resistência do NTC

    return r_ntc;
}
```

1. **Leitura do ADC**:
   - Lê o valor do ADC (0 a 4095) usando `adc1_get_raw`.

2. **Cálculo da Tensão**:
   - Converte o valor lido do ADC em tensão (0 a 3.3V).

3. **Cálculo da Resistência**:
   - Usa um divisor de tensão para calcular a resistência do NTC.

---

### **Função `convert_resistance_to_temperature`**

```c
float convert_resistance_to_temperature(float r_ntc) {
    // Equação de Steinhart-Hart
    float steinhart = log(r_ntc / R0) / BETA; // ln(R/R0) / B
    steinhart += 1.0 / T0;                   // + 1/T0
    steinhart = 1.0 / steinhart;             // 1 / (ln(R/R0) / B + 1/T0)

    // Converte de Kelvin para Celsius
    float temperature = steinhart - 273.15;

    return temperature;
}
```

1. **Equação de Steinhart-Hart**:
   - Usa a equação de Steinhart-Hart para converter a resistência do NTC em temperatura em Kelvin.
   - A equação é:
     \[
     \frac{1}{T} = \frac{1}{T_0} + \frac{1}{B} \cdot \ln\left(\frac{R}{R_0}\right)
     \]
   - Onde:
     - \( T \) é a temperatura em Kelvin.
     - \( T_0 \) é a temperatura de referência em Kelvin (25°C = 298.15 K).
     - \( B \) é o coeficiente Beta do NTC.
     - \( R \) é a resistência do NTC.
     - \( R_0 \) é a resistência do NTC a 25°C.

2. **Conversão para Celsius**:
   - Converte a temperatura de Kelvin para Celsius subtraindo 273.15.

---

### **Função Principal `app_main`**

```c
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
```

1. **Inicialização**:
   - Configura o ADC usando a função `adc_init`.

2. **Loop Principal**:
   - Lê a resistência do NTC usando `read_ntc_resistance`.
   - Converte a resistência em temperatura usando `convert_resistance_to_temperature`.
   - Exibe os valores de resistência e temperatura no terminal serial.
   - Aguarda 1 segundo antes de repetir o processo usando `vTaskDelay`.

---

### **Fluxo de Funcionamento**

1. **Inicialização**:
   - Configura o ADC para ler o valor do sensor NTC.

2. **Leitura do NTC**:
   - Lê o valor do ADC e calcula a resistência do NTC.

3. **Conversão para Temperatura**:
   - Converte a resistência do NTC em temperatura usando a equação de Steinhart-Hart.

4. **Exibição dos Valores**:
   - Exibe a resistência e a temperatura no terminal serial.

5. **Repetição**:
   - O processo é repetido a cada 1 segundo.

---

### **Exemplo de Saída**

```
Resistência do NTC: 10000.00 Ω, Temperatura: 25.00 °C
Resistência do NTC: 8000.00 Ω, Temperatura: 30.00 °C
Resistência do NTC: 12000.00 Ω, Temperatura: 20.00 °C
```

---

### **Conclusão**

Este código é uma implementação simples e eficaz de um monitor de temperatura usando um sensor NTC no ESP32. Ele pode ser facilmente expandido para incluir mais funcionalidades, como notificações remotas ou integração com outros dispositivos. Se precisar de mais detalhes ou ajuda, é só perguntar! 😊
