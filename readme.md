# Mini Projetos com ESP-IDF

Este repositório contém uma coleção de mini projetos desenvolvidos com ESP-IDF para estudo de sistemas embarcados. Cada projeto aborda conceitos essenciais como interrupções, comunicação serial, sensores e controle de periféricos.

## Projetos

### 1. Controle de LED com Botão (Interrupções)
- Utiliza uma interrupção externa para detectar o pressionamento de um botão e alternar o estado de um LED.

### 2. Terminal Serial Interativo (Comunicação Serial)
- Cria um menu via UART para interagir com o ESP32, permitindo enviar comandos para ligar/desligar LEDs e exibir valores de sensores.

### 3. Leitura Suavizada do ADC (Filtro de Média Móvel)
- Lê o valor de um potenciômetro (ou outro sensor analógico) e aplica um filtro de média móvel para suavizar as leituras.

### 4. Controle de Brilho de LED com PWM
- Usa PWM para controlar o brilho de um LED ou a velocidade de um motor.

### 5. Detector de Luz com Sensor LDR
- Lê o valor do LDR e ajusta o brilho de um LED proporcionalmente à luminosidade.

### 6. Alarme de Movimento com Sensor PIR
- Detecta movimento com um sensor PIR e ativa um LED ou buzzer como alerta.

### 7. Monitor de Temperatura com Sensor NTC
- Lê a resistência do NTC via ADC e converte para temperatura.

### 8. Controle de Servomotor com Analog Joystick
- Usa um joystick analógico para controlar o ângulo de um servomotor.

### 9. Indicador de Nível com LED Bar Graph
- Exibe o valor de um sensor analógico (ex: potenciômetro) utilizando um LED Bar Graph para indicar níveis.
