# ✈️ Plane Flight Controller

Firmware e eletrônica para um sistema de controle de voo de avião rádio controlado.

Este repositório contém o firmware para um sistema embarcado de controle de voo para um avião. O projeto utiliza PlatformIO para compilação e upload, e implementa o controle de superfícies de voo (ailerons, profundor e leme) através de um joystick, com logging de dados em cartão SD e exibição de status em um display LCD.

## ⚙️ Visão geral do firmware

Principais módulos incluídos:
- Leitura de Joystick para controle manual.
- Controle de múltiplos servo motores para as superfícies de voo.
- Registro de estados de voo em cartão SD (`SDCardLogger`).
- Exibição de status em tempo real em um display 16x2 (`display`).
- Gerenciamento de tarefas concorrentes com FreeRTOS.

### Componentes Principais

- **Microcontrolador**: ESP32 Dev Module
- **Controle**: Joystick analógico com botão
- **Atuadores**: Servo motores para Ailerons, Profundor e Leme
- **Armazenamento**: Cartão SD para logging de dados
- **Display**: LCD 16x2 com interface I2C

## ✅ Requisitos

- PlatformIO (versão estável recomendada) instalada como extensão do VS Code ou via CLI.
- Toolchain para a placa ESP32 (o PlatformIO instalará automaticamente).

## 📌 Pinagem do MCU

| GPIO (ESP32) | Sinal / Identificador | Descrição / Conexão |
|---:|---|---|
| 21 | DISPLAY_SDA_PIN | I2C SDA para o Display LCD |
| 22 | DISPLAY_SCL_PIN | I2C SCL para o Display LCD |
| 34 | JOYSTICK_ADC_X_PIN | Eixo X do Joystick (Analógico) |
| 35 | JOYSTICK_ADC_Y_PIN | Eixo Y do Joystick (Analógico) |
| 27 | JOYSTICK_BUTTON_PIN | Botão do Joystick (Digital com interrupção) |
| 26 | AILERON_LEFT_SERVO_PIN | Servo do Aileron Esquerdo |
| 25 | AILERON_RIGHT_SERVO_PIN | Servo do Aileron Direito |
| 18 | ELEVATOR_LEFT_SERVO_PIN| Servo do Profundor Esquerdo |
| 19 | ELEVATOR_RIGHT_SERVO_PIN| Servo do Profundor Direito |
| 5  | RUDDER_SERVO_PIN | Servo do Leme |
| 15 | SD_CS | Chip Select (CS) do Cartão SD |
| 14 | SPI SCK | Clock SPI para Cartão SD |
| 12 | SPI MISO | MISO SPI para Cartão SD |
| 13 | SPI MOSI | MOSI SPI para Cartão SD |

## 📦 Bibliotecas Utilizadas

- **Arduino framework**
- **FreeRTOS** (integrado ao ESP-IDF do Arduino)
- **LiquidCrystal_I2C** v1.1.4 - Para controle do display LCD.
- Bibliotecas locais (`servo.hpp`, `display.hpp`, `SDCardLogger.h`)

## 🗂️ Estrutura do Projeto

```
./
├── firmware/
│   ├── platformio.ini
│   ├── include/
│   │   ├── display.hpp
│   │   ├── README
│   │   ├── SDCardLogger.h
│   │   └── servo.hpp
│   ├── lib/
│   │   └── README
│   ├── src/
│   │   ├── display.cpp
│   │   ├── main.cpp
│   │   ├── SDCardLogger.cpp
│   │   └── servo.cpp
│   └── test/
│       └── README
└── README.md
```

- `firmware/`: Contém todo o código fonte do projeto embarcado.
	- `platformio.ini`: Arquivo de configuração do PlatformIO, definindo a placa, framework e dependências.
	- `include/`: Arquivos de cabeçalho (`.h`, `.hpp`) para os módulos do projeto.
	- `src/`: Arquivos de implementação (`.cpp`) e o `main.cpp`, que contém a lógica principal e a configuração do sistema.
	- `lib/`: Diretório para bibliotecas locais ou de terceiros.

## 🚀 Estados de Voo

O sistema opera com base nos seguintes estados de voo, determinados pela posição do joystick:

- **CRUISE**: Estado neutro, voo de cruzeiro.
- **PITCH_UP**: Movimento de arfagem para cima (cabrar).
- **PITCH_DOWN**: Movimento de arfagem para baixo (picar).
- **YAW_RIGHT**: Movimento de guinada para a direita.
- **YAW_LEFT**: Movimento de guinada para a esquerda.
- **ROLL_RIGHT**: Movimento de rolagem para a direita.
- **ROLL_LEFT**: Movimento de rolagem para a esquerda.

O botão do joystick alterna o controle do eixo X entre os modos **YAW** (leme) e **ROLL** (ailerons).

## 📊 Taxa de Amostragem

- **Leitura do Joystick**: A cada 70ms (controlado por um software timer do FreeRTOS).
- **Escrita no Display**: A cada 100ms.
- **Gravação no Cartão SD**: A cada 100ms, com um flush para o disco a cada 10 registros.

## 🏃 Tasks FreeRTOS

O firmware utiliza o FreeRTOS para gerenciar tarefas concorrentes. A seguir, a descrição das tasks principais:

| Task | Core | Prio | Descrição |
|---|---|---|---|
| `vJoystickReadTask` | 1 (APP) | 1 | Lê o joystick a cada 70ms (via timer), determina o estado de voo e envia para as outras tasks. |
| `vControlTask` | 0 (PRO) | 1 | Recebe o estado de voo e os valores do joystick para controlar os ângulos dos servos. |
| `vDisplayWriteTask` | 0 (PRO) | 2 | Recebe o estado de voo e o exibe no display LCD 16x2. |
| `vSDCardSaveTask` | 1 (APP) | 2 | Recebe o estado de voo e o registra em um arquivo CSV no cartão SD com um timestamp. |

## ⚙️ Compilação e Upload

### Pré-requisitos
- PlatformIO CLI ou extensão VS Code instalada.
- Placa ESP32 Dev Module.

Para compilar e fazer o upload, utilize os comandos do PlatformIO no seu ambiente de desenvolvimento:
- **Build**: `pio run`
- **Upload**: `pio run --target upload`
- **Monitor Serial**: `pio device monitor`
