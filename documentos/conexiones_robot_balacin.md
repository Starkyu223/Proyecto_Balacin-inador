# Conexiones completas — Robot Balancín

## Arquitectura del sistema

| MCU | Rol |
|---|---|
| ESP32-S3-WROOM N16R8 | Control: motores, encoders, MPU-6050, botón |
| GOOUUU ESP32-S3-CAM V1.5 (OV3660) | Visión artificial y cámara (integrar después) |

---

## Redes de alimentación

| Red | Origen | Destino |
|---|---|---|
| VBAT | Batería + | Entrada conversor DC/DC |
| GND | Batería − | GND común de todo el sistema |
| 5V | Salida conversor DC/DC | Pin 5V del DevKit (pin 21), VM del TB6612 |
| 3.3V | Pin 3V3 del DevKit (pin 1) | VCC del TB6612, VCC del MPU-6050, VCC encoders, STBY del TB6612, pull-ups |

> El conversor DC/DC no necesita pines GPIO. Solo conexión de potencia.

---

## ESP32-S3-DevKitM-1 (símbolo en esquemático)

| Pin esquemático | GPIO / Función | Conecta a | Notas |
|---|---|---|---|
| 1 | 3V3 | Rail 3.3V del sistema | Salida regulada |
| 21 | 5V | Salida conversor DC/DC | Entrada de alimentación al DevKit |
| 22 | GND | GND común | |
| 43 | RST | — | Sin conectar |
| 3 | GPIO1 | TB6612 → AIN1 | |
| 4 | GPIO2 | TB6612 → AIN2 | |
| 16 | GPIO14 | TB6612 → PWMA | PWM canal A |
| 17 | GPIO15 | TB6612 → BIN1 | |
| 18 | GPIO16 | TB6612 → BIN2 | |
| 19 | GPIO17 | TB6612 → PWMB | PWM canal B |
| 20 | GPIO18 | Encoder Motor A → CH_A | |
| 27 | GPIO21 | Encoder Motor A → CH_B | |
| 34 | GPIO38 | Encoder Motor B → CH_A | |
| 35 | GPIO39 | Encoder Motor B → CH_B | |
| 23 | GPIO47 | MPU-6050 → SDA | + pull-up 4.7kΩ a 3.3V en PCB |
| 24 | GPIO48 | MPU-6050 → SCL | + pull-up 4.7kΩ a 3.3V en PCB |
| 36 | GPIO40 | Botón Start | + pull-down 10kΩ a GND en PCB |
| 39 | TX (GPIO43) | GOOUUU ESP32 → RX | Reservado, conectar al integrar cámara |
| 40 | RX (GPIO44) | GOOUUU ESP32 → TX | Reservado, conectar al integrar cámara |

### Pines NO usar (dejar sin conectar)

| Pin esquemático | GPIO | Motivo |
|---|---|---|
| 2 | GPIO0 | Strapping BOOT |
| 5 | GPIO3 | Strapping JTAG |
| 41 | GPIO45 | Strapping VDD_SPI |
| 42 | GPIO46 | Strapping |
| 31–33 | GPIO35, GPIO36, GPIO37 | Octal PSRAM interno |
| 28–30 | GPIO26, GPIO33, GPIO34 | Flash interno / sin exponer |
| 25–26 | GPIO19, GPIO20 | USB D+/D− |

---

## TB6612FNG — Driver de motores

| Pin TB6612 | Conecta a | Notas |
|---|---|---|
| VCC | 3.3V | Alimentación lógica |
| VM | 5V (conversor DC/DC) | Alimentación motores |
| GND | GND común | |
| STBY | 3.3V directo | Sin GPIO — siempre habilitado |
| AIN1 | GPIO1 (DevKit pin 3) | Dirección motor A |
| AIN2 | GPIO2 (DevKit pin 4) | Dirección motor A |
| PWMA | GPIO14 (DevKit pin 16) | Velocidad motor A |
| BIN1 | GPIO15 (DevKit pin 17) | Dirección motor B |
| BIN2 | GPIO16 (DevKit pin 18) | Dirección motor B |
| PWMB | GPIO17 (DevKit pin 19) | Velocidad motor B |
| AO1 | Motor A → cable M+ | Salida potencia motor A |
| AO2 | Motor A → cable M− | Salida potencia motor A |
| BO1 | Motor B → cable M+ | Salida potencia motor B |
| BO2 | Motor B → cable M− | Salida potencia motor B |

### Componentes pasivos obligatorios del TB6612 en PCB

| Componente | Valor | Ubicación |
|---|---|---|
| C1 | 100nF cerámico | Entre VM y GND, pegado al CI |
| C2 | 100nF cerámico | Entre VCC y GND, pegado al CI |

---

## Motor A — Reductor con encoder (6 cables)

| Cable motor | Conecta a |
|---|---|
| M+ (potencia) | TB6612 → AO1 |
| M− (potencia) | TB6612 → AO2 |
| VCC encoder | 3.3V |
| GND encoder | GND común |
| CH_A | GPIO18 (DevKit pin 20) |
| CH_B | GPIO21 (DevKit pin 27) |

---

## Motor B — Reductor con encoder (6 cables)

| Cable motor | Conecta a |
|---|---|
| M+ (potencia) | TB6612 → BO1 |
| M− (potencia) | TB6612 → BO2 |
| VCC encoder | 3.3V |
| GND encoder | GND común |
| CH_A | GPIO38 (DevKit pin 34) |
| CH_B | GPIO39 (DevKit pin 35) |

---

## MPU-6050

| Pin MPU-6050 | Conecta a | Notas |
|---|---|---|
| VCC | 3.3V | |
| GND | GND común | |
| SDA | GPIO47 (DevKit pin 23) | Pull-up 4.7kΩ a 3.3V |
| SCL | GPIO48 (DevKit pin 24) | Pull-up 4.7kΩ a 3.3V |
| AD0 | GND | Dirección I2C = 0x68 |
| INT | Sin conectar | Opcional para uso futuro |
| XDA | Sin conectar | |
| XCL | Sin conectar | |

### Componentes pasivos obligatorios del MPU-6050 en PCB

| Componente | Valor | Ubicación |
|---|---|---|
| C3 | 100nF cerámico | Entre VCC y GND del MPU, pegado al CI |
| R1 | 4.7kΩ | Pull-up SDA → 3.3V |
| R2 | 4.7kΩ | Pull-up SCL → 3.3V |

> Si usas módulo GY-521, ya trae pull-ups y capacitor. No los duplices.

---

## Botón Start

```
3.3V ──[Botón]──┬── GPIO40 (DevKit pin 36)
                │
              [10kΩ]
                │
               GND
```

| Componente | Valor | Conexión |
|---|---|---|
| Botón | — | Entre 3.3V y GPIO40 |
| R3 | 10kΩ | Entre GPIO40 y GND (pull-down) |

Comportamiento: GPIO40 lee `0` en reposo, `1` al presionar.

---

## Conversor DC/DC reductor

| Pin conversor | Conecta a |
|---|---|
| IN+ | Batería + |
| IN− | Batería − / GND |
| OUT+ | 5V → DevKit pin 21, VM del TB6612 |
| OUT− | GND común |

> Configurar salida en 5V antes de conectar. No requiere pines GPIO.

---

## UART entre ESP32-S3-WROOM y GOOUUU (integrar después)

| WROOM | GOOUUU ESP32-S3-CAM | Notas |
|---|---|---|
| TX GPIO43 (pin 39) | RX de la GOOUUU | |
| RX GPIO44 (pin 40) | TX de la GOOUUU | |
| GND | GND | GND compartido obligatorio |

> Dejar las pistas enrutadas en la PCB aunque no se usen todavía.

---

## Resumen de componentes pasivos para PCB

| Ref | Valor | Cantidad | Función |
|---|---|---|---|
| C1, C2 | 100nF cerámico | 2 | Desacople VM y VCC del TB6612 |
| C3 | 100nF cerámico | 1 | Desacople VCC del MPU-6050 |
| R1, R2 | 4.7kΩ | 2 | Pull-up I2C (SDA y SCL) |
| R3 | 10kΩ | 1 | Pull-down botón Start |

---

## Notas críticas para el layout de PCB

1. **GND unificado** — un único plano de tierra para todo. Sin separar GND de señal y GND de potencia en planos distintos a menos que tengas experiencia en EMC.
2. **Pistas de motores anchas** — AO1, AO2, BO1, BO2 y VM mínimo 1mm, ideal 2mm.
3. **Capacitores de desacople pegados al CI** — lo más cerca posible del pin VCC de cada componente.
4. **Pistas de encoders e I2C alejadas de las pistas de potencia de motores** — el ruido de los motores se acopla si van paralelas.
5. **STBY del TB6612 directo a 3.3V** — traza corta, sin resistencia.
6. **AD0 del MPU directo a GND** — fija la dirección I2C en 0x68.
