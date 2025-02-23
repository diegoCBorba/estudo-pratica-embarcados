### [link wokwi](https://wokwi.com/projects/423713477227640833)

### **Estrutura do Código**

O código está dividido em três partes principais:
1. **Configuração do botão**: Configura o pino do botão como entrada com interrupção.
2. **Configuração do LED**: Configura o pino do LED como saída.
3. **Loop principal**: Verifica se o botão foi pressionado e alterna o estado do LED.

Abaixo, detalhamos cada parte do código.

---

### **Inclusão de Bibliotecas**

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
```

- **`stdio.h`**: Biblioteca padrão para funções de entrada e saída, como `printf`.
- **`freertos/FreeRTOS.h`**: Biblioteca do FreeRTOS, que gerencia tarefas e delays.
- **`freertos/task.h`**: Contém funções para criar e gerenciar tarefas.
- **`driver/gpio.h`**: Biblioteca do ESP-IDF para configurar e controlar GPIOs.

---

### **Definição de Constantes e Variáveis**

```c
#define LED_RED GPIO_NUM_2
#define BTN_RED GPIO_NUM_13
#define DELAY_TIME 200

volatile bool button_pressed = false;
```

- **`LED_RED`**: Define o pino GPIO onde o LED está conectado (GPIO 2).
- **`BTN_RED`**: Define o pino GPIO onde o botão está conectado (GPIO 13).
- **`DELAY_TIME`**: Define o tempo de delay em milissegundos (200 ms).
- **`button_pressed`**: Variável global `volatile` que indica se o botão foi pressionado. O uso de `volatile` é importante para garantir que o compilador não otimize o acesso a essa variável, já que ela é modificada em uma interrupção.

---

### **Função de Tratamento de Interrupção**

```c
static void gpio_isr_handler(void* arg) 
{
    button_pressed = true;
}
```

- **`gpio_isr_handler`**: Esta função é chamada quando ocorre uma interrupção no pino do botão.
- **`button_pressed = true`**: Define a variável `button_pressed` como `true` para indicar que o botão foi pressionado.

---

### **Configuração do Botão**

```c
void button_config()
{
    gpio_install_isr_service(0);
    printf("configuring button\n");
    gpio_reset_pin(BTN_RED);
    gpio_set_direction(BTN_RED, GPIO_MODE_INPUT);
    gpio_pullup_en(BTN_RED);
    gpio_set_intr_type(BTN_RED, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(BTN_RED, gpio_isr_handler, NULL);
    printf("config complete\n");
}
```

- **`gpio_install_isr_service(0)`**: Instala o serviço de interrupção para GPIOs. O parâmetro `0` define a prioridade padrão.
- **`gpio_reset_pin(BTN_RED)`**: Reseta o pino do botão para o estado padrão.
- **`gpio_set_direction(BTN_RED, GPIO_MODE_INPUT)`**: Configura o pino do botão como entrada.
- **`gpio_pullup_en(BTN_RED)`**: Habilita o resistor pull-up interno para o pino do botão, garantindo que ele tenha um estado definido (HIGH) quando não pressionado.
- **`gpio_set_intr_type(BTN_RED, GPIO_INTR_POSEDGE)`**: Configura a interrupção para ser acionada na borda de subida (quando o botão é pressionado).
- **`gpio_isr_handler_add(BTN_RED, gpio_isr_handler, NULL)`**: Associa a função `gpio_isr_handler` ao pino do botão.

---

### **Configuração do LED**

```c
void led_config()
{
    gpio_reset_pin(LED_RED);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
}
```

- **`gpio_reset_pin(LED_RED)`**: Reseta o pino do LED para o estado padrão.
- **`gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT)`**: Configura o pino do LED como saída.

---

### **Função Principal (`app_main`)**

```c
void app_main()
{   
    uint8_t led_value = 0;

    button_config();
    led_config();

    while (1) {
        if (button_pressed) {
            printf("*\n");
            button_pressed = false;
            led_value = !led_value;
            gpio_set_level(LED_RED, led_value);
        }
        vTaskDelay(DELAY_TIME / portTICK_PERIOD_MS);
    }
}
```

- **`uint8_t led_value = 0`**: Variável local para armazenar o estado atual do LED (0 = desligado, 1 = ligado).
- **`button_config()`**: Chama a função para configurar o botão.
- **`led_config()`**: Chama a função para configurar o LED.
- **`while (1)`**: Loop infinito que verifica se o botão foi pressionado.
  - **`if (button_pressed)`**: Verifica se a variável `button_pressed` foi definida como `true` na interrupção.
    - **`printf("*\n")`**: Imprime um asterisco no console para indicar que o botão foi pressionado.
    - **`button_pressed = false`**: Reseta a variável `button_pressed` para `false`.
    - **`led_value = !led_value`**: Alterna o estado do LED.
    - **`gpio_set_level(LED_RED, led_value)`**: Atualiza o estado do LED no pino GPIO.
  - **`vTaskDelay(DELAY_TIME / portTICK_PERIOD_MS)`**: Aguarda 200 ms antes de verificar novamente. Isso evita o uso excessivo da CPU.

---

### **Fluxo de Funcionamento**

1. O botão é configurado como entrada com interrupção na borda de subida.
2. O LED é configurado como saída.
3. Quando o botão é pressionado, a interrupção é acionada, e a função `gpio_isr_handler` define `button_pressed` como `true`.
4. No loop principal, o código verifica se `button_pressed` é `true`. Se for, ele alterna o estado do LED e imprime um asterisco no console.
5. O loop principal espera 200 ms antes de verificar novamente.

---

### **Considerações Importantes**

1. **Debounce do Botão**:
   - O código não implementa debounce, o que pode causar múltiplas detecções de pressionamento devido ao ruído mecânico do botão. Para corrigir isso, você pode adicionar um delay ou usar um timer para ignorar múltiplas interrupções em um curto período.

2. **Uso de `volatile`**:
   - A variável `button_pressed` é declarada como `volatile` porque é modificada em uma interrupção e lida no loop principal. Isso garante que o compilador não otimize o acesso a essa variável.

3. **Interrupções**:
   - Interrupções são usadas para detectar eventos rápidos, como o pressionamento de um botão, sem a necessidade de polling constante no loop principal.

4. **GPIOs**:
   - Certifique-se de que os pinos GPIO usados (`LED_RED` e `BTN_RED`) estejam corretos para o seu hardware.

---

### **Melhorias Possíveis**

1. **Debounce**:
   - Adicionar um mecanismo de debounce para evitar múltiplas detecções de pressionamento.

2. **Controle de Estado**:
   - Implementar um estado mais complexo, como um contador de pressionamentos ou um controle de brilho do LED.

3. **Uso de Timers**:
   - Usar timers para controlar o tempo de resposta ou implementar funcionalidades adicionais.
