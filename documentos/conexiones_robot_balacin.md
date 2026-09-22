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
| VBAT | Batería + | VM del TB6612, IN+ del conversor DC/DC |
| GND | Batería − | GND común de todo el sistema |
| 5V | Salida conversor DC/DC | Pin 5V del WROOM (pin 21), Pin 5V de la GOOUUU (pin 20) |
| 3.3V | Pin 3V3 del WROOM (pin 1) | VCC del TB6612, VCC encoders, VCC MPU-6050, STBY del TB6612, pull-ups |

> El conversor DC/DC no necesita pines GPIO. Solo conexión de potencia.
> Los motores se alimentan directo de la batería a través de VM del TB6612.

---

## ESP32-S3-WROOM N16R8 — Control

### Pines utilizados

| Pin esquemático | GPIO / Función | Conecta a | Notas |
|---|---|---|---|
| 1 | 3V3 | Rail 3.3V del sistema | Salida regulada, max 500mA |
| 21 | 5V | Salida conversor DC/DC | Entrada de alimentación al DevKit |
| 22 | GND | GND común | |
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
| 23 | GPIO47 | MPU-6050 → SDA | + pull-up 4.7kΩ a 3.3V |
| 24 | GPIO48 | MPU-6050 → SCL | + pull-up 4.7kΩ a 3.3V |
| 36 | GPIO40 | Botón Start | + pull-down 10kΩ a GND |
| 39 | TX / GPIO43 | GOOUUU → RX (pin 39) | Reservado, conectar al integrar cámara |
| 40 | RX / GPIO44 | GOOUUU → TX (pin 40) | Reservado, conectar al integrar cámara |

### Pines NO usar — dejar sin conectar

| Pin esquemático | GPIO | Motivo |
|---|---|---|
| 2 | GPIO0 | Strapping BOOT |
| 5 | GPIO3 | Strapping JTAG |
| 41 | GPIO45 | Strapping VDD_SPI |
| 42 | GPIO46 | Strapping |
| 31–33 | GPIO35, GPIO36, GPIO37 | Octal PSRAM interno |
| 28–30 | GPIO26, GPIO33, GPIO34 | Flash interno |
| 25–26 | GPIO19, GPIO20 | USB D+/D− |

---

## GOOUUU ESP32-S3-CAM V1.5 — Visión artificial

### Pines ocupados por la cámara OV3660 (internos, no tocar)

| Pin placa | GPIO | Función |
|---|---|---|
| 3 | GPIO4 | CAM SIOD |
| 4 | GPIO5 | CAM SIOC |
| 5 | GPIO6 | CAM VSYNC |
| 6 | GPIO7 | CAM HREF |
| 7 | GPIO15 | CAM XCLK |
| 8 | GPIO16 | CAM D0 |
| 9 | GPIO17 | CAM D1 |
| 10 | GPIO18 | CAM D2 |
| 11 | GPIO8 | CAM D3 |
| 12 | GPIO3 | CAM D4 |
| 13 | GPIO46 | CAM D5 |
| 14 | GPIO9 | CAM D6 |
| 15 | GPIO10 | CAM D7 |
| 16 | GPIO11 | CAM PCLK |

### Pines con función fija de la placa (no tocar)

| Pin placa | GPIO | Función |
|---|---|---|
| 1 | 3.3V | Alimentación salida |
| 20 | 5V | Alimentación entrada |
| 21 | GND | Tierra |
| 2 | EN/RST | Reset (dejar sin conectar o botón a GND) |
| 26 | GPIO48 | WSLED — LED RGB onboard |
| 23 | GPIO20 | USB D+ |
| 22 | GPIO19 | USB D− |
| 29 | GPIO35 | PSRAM |
| 30 | GPIO36 | PSRAM |
| 31 | GPIO37 | PSRAM |
| 28 | GPIO0 | BOOT |
| 27 | GPIO45 | Strapping — no conectar |

### Pines de comunicación con el WROOM

| Pin placa | GPIO | Conecta a |
|---|---|---|
| 40 | TX / GPIO43 | WROOM RX (GPIO44, pin 40) |
| 39 | RX / GPIO44 | WROOM TX (GPIO43, pin 39) |
| 21 | GND | GND común con el WROOM — obligatorio |

### Pines libres para regleta (uso futuro)

| Pin placa | GPIO |
|---|---|
| 38 | GPIO1 |
| 37 | GPIO2 |
| 36 | GPIO42 |
| 35 | GPIO41 |
| 34 | GPIO40 |
| 33 | GPIO39 |
| 32 | GPIO38 |
| 25 | GPIO47 |
| 24 | GPIO21 |
| 19 | GPIO14 |
| 18 | GPIO13 |
| 17 | GPIO12 |

---

## TB6612FNG — Driver de motores

| Pin TB6612 | Conecta a | Notas |
|---|---|---|
| VCC | 3.3V | Alimentación lógica |
| VM | VBAT (batería directa) | Alimentación motores |
| GND | GND común | |
| STBY | 3.3V directo | Sin GPIO — siempre habilitado |
| AIN1 | GPIO1 (WROOM pin 3) | Dirección motor A |
| AIN2 | GPIO2 (WROOM pin 4) | Dirección motor A |
| PWMA | GPIO14 (WROOM pin 16) | Velocidad motor A |
| BIN1 | GPIO15 (WROOM pin 17) | Dirección motor B |
| BIN2 | GPIO16 (WROOM pin 18) | Dirección motor B |
| PWMB | GPIO17 (WROOM pin 19) | Velocidad motor B |
| AO1 | Motor A → cable M+ | Salida potencia motor A |
| AO2 | Motor A → cable M− | Salida potencia motor A |
| BO1 | Motor B → cable M+ | Salida potencia motor B |
| BO2 | Motor B → cable M− | Salida potencia motor B |

### Componentes pasivos obligatorios del TB6612

| Componente | Valor | Ubicación | Motivo |
|---|---|---|---|
| C1 | 100nF cerámico | Entre VM y GND, pegado al CI | Absorbe picos de corriente de los motores |
| C2 | 100nF cerámico | Entre VCC y GND, pegado al CI | Estabiliza la lógica del driver |

---

## Motor A — Reductor con encoder (6 cables)

| Cable | Conecta a |
|---|---|
| M+ (potencia) | TB6612 → AO1 |
| M− (potencia) | TB6612 → AO2 |
| VCC encoder | 3.3V |
| GND encoder | GND común |
| CH_A | GPIO18 (WROOM pin 20) |
| CH_B | GPIO21 (WROOM pin 27) |

---

## Motor B — Reductor con encoder (6 cables)

| Cable | Conecta a |
|---|---|
| M+ (potencia) | TB6612 → BO1 |
| M− (potencia) | TB6612 → BO2 |
| VCC encoder | 3.3V |
| GND encoder | GND común |
| CH_A | GPIO38 (WROOM pin 34) |
| CH_B | GPIO39 (WROOM pin 35) |

---

## MPU-6050

| Pin MPU-6050 | Conecta a | Notas |
|---|---|---|
| VCC | 3.3V | |
| GND | GND común | |
| SDA | GPIO47 (WROOM pin 23) | Pull-up 4.7kΩ a 3.3V |
| SCL | GPIO48 (WROOM pin 24) | Pull-up 4.7kΩ a 3.3V |
| AD0 | GND | Dirección I2C fija = 0x68 |
| INT | Sin conectar | Opcional para uso futuro |
| XDA | Sin conectar | |
| XCL | Sin conectar | |

### Componentes pasivos obligatorios del MPU-6050

| Componente | Valor | Ubicación |
|---|---|---|
| C3 | 100nF cerámico | Entre VCC y GND del MPU, pegado al CI |
| R1 | 4.7kΩ | Pull-up SDA → 3.3V |
| R2 | 4.7kΩ | Pull-up SCL → 3.3V |

> Si usas módulo GY-521 ya trae pull-ups y capacitor incluidos. No los duplices.

---

## Botón Start

```
3.3V ──[Botón]──┬── GPIO40 (WROOM pin 36)
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
| OUT+ | 5V → WROOM pin 21, GOOUUU pin 20 |
| OUT− | GND común |

> Configurar salida en 5V antes de conectar. No requiere pines GPIO.

---

## Resumen de componentes pasivos para PCB

| Ref | Valor | Cantidad | Función |
|---|---|---|---|
| C1 | 100nF cerámico | 1 | Desacople VM del TB6612 |
| C2 | 100nF cerámico | 1 | Desacople VCC del TB6612 |
| C3 | 100nF cerámico | 1 | Desacople VCC del MPU-6050 |
| R1 | 4.7kΩ | 1 | Pull-up SDA |
| R2 | 4.7kΩ | 1 | Pull-up SCL |
| R3 | 10kΩ | 1 | Pull-down botón Start |

---

## Notas críticas para el layout de PCB

1. **GND unificado** — un único plano de tierra para todo el sistema. WROOM, GOOUUU, TB6612, motores y MPU en el mismo plano.
2. **VM del TB6612 va directo a la batería** — pista ancha, mínimo 2mm. Es la línea de mayor corriente del sistema.
3. **Pistas de potencia de motores anchas** — AO1, AO2, BO1, BO2 mínimo 1mm, ideal 2mm.
4. **Capacitores de desacople pegados al CI** — lo más cerca posible del pin VCC/VM de cada componente. Sin esto el ruido de los motores llega al ESP32 y causa resets.
5. **Pistas de encoders e I2C alejadas de pistas de potencia** — el ruido de los motores se acopla si van paralelas.
6. **STBY del TB6612 directo a 3.3V** — traza corta, sin resistencia.
7. **AD0 del MPU directo a GND** — fija la dirección I2C en 0x68.
8. **GND compartido entre WROOM y GOOUUU** — obligatorio para que la UART funcione correctamente.
9. **Dejar pistas TX/RX enrutadas en PCB** aunque la GOOUUU no se conecte todavía.
