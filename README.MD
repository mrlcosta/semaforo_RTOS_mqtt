# Semáforo IoT com RTOS e MQTT

Este projeto implementa um semáforo inteligente utilizando o Raspberry Pi Pico W, com sistema operacional em tempo real (FreeRTOS) e comunicação MQTT para controle remoto.

## Características

- Controle de semáforo com modo diurno e noturno
- Display OLED para feedback visual
- Matriz de LED RGB para indicação visual
- Buzzer para alertas sonoros
- Controle remoto via MQTT
- Interface física com botões
- Sistema operacional em tempo real (FreeRTOS)

## Hardware Necessário

- Raspberry Pi Pico W
- Display OLED SSD1306 (I2C)
- Matriz de LED RGB 5x5
- Buzzer
- 2 LEDs (Verde e Vermelho)
- 2 Botões
- Resistor de pull-up (10kΩ) para os botões

## Conexões

### Display OLED (I2C)
- SDA: GPIO4
- SCL: GPIO5

### Matriz LED
- Data: GPIO6

### LEDs
- LED Verde: GPIO7
- LED Vermelho: GPIO8

### Buzzer
- PWM: GPIO9

### Botões
- Botão A: GPIO10
- Botão B: GPIO11

## Configuração

1. Clone o repositório:
```bash
git clone [URL_DO_REPOSITORIO]
cd semaforo_RTOS_mqtt
```

2. Configure o ambiente de desenvolvimento:
   - Instale o SDK do Raspberry Pi Pico
   - Configure o CMake
   - Compile o projeto

3. Configure as credenciais WiFi e MQTT:
   - Edite o arquivo `config.h` com suas credenciais WiFi
   - Configure o broker MQTT no arquivo `main.c`

## Uso

### Controle Local
- Botão A: Alterna entre modo diurno e noturno
- Botão B: Entra no modo BOOTSEL

### Controle Remoto via MQTT

O semáforo pode ser controlado remotamente através de mensagens MQTT:

#### Tópicos MQTT

1. `semaforo/modo`
   - Mensagem: `toggle`
   - Ação: Alterna entre modo diurno e noturno

2. `semaforo/buzzer`
   - Mensagem: `toggle`
   - Ação: Ativa/desativa alertas sonoros

3. `semaforo/estado`
   - Formato: `modo,sonoro`
   - Exemplo: `diurno,sonoro` ou `noturno,mudo`
   - Publicado automaticamente a cada 5 segundos

### Exemplo de Uso MQTT

```bash
# Alternar modo
mosquitto_pub -h 192.168.1.157 -u muriel -P muriel -t "semaforo/modo" -m "toggle"

# Alternar buzzer
mosquitto_pub -h 192.168.1.157 -u muriel -P muriel -t "semaforo/buzzer" -m "toggle"

# Monitorar estado
mosquitto_sub -h 192.168.1.157 -u muriel -P muriel -t "semaforo/estado"
```

## Estados do Semáforo

### Modo Diurno
1. Verde: "Prossiga"
2. Amarelo: "Atenção"
3. Vermelho: "Pare"

### Modo Noturno
- Amarelo piscante: "Preste atenção"


## demonstração
- link do video: https://www.youtube.com/watch?v=tGGELkMGMgY&ab_channel=murielcosta