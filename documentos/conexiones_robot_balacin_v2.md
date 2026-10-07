# Conexiones completas — Robot Balancín (v2, corregida)

Cambios respecto a la v1:
- El control pasa a una **ESP32-S3-DevKitC-1 con módulo WROOM-1 N16R8** (antes el diseño estaba pensado con la distribución de la DevKitM-1 / MINI-1).
- Los números de pin de este documento son los del **símbolo U3 del esquemático**, no la posición física en la placa.
- Se reasignaron los GPIO para acortar pistas y se evitan GPIO48 y GPIO38 (LED RGB según la versión de la placa).
- Se agregan headers de pines libres (H4, H8, H5).
- Reglas de PCB ajustadas para baquelita casera (quemado).

---

## Arquitectura del sistema

| MCU | Rol |
|---|---|
| ESP32-S3-DevKitC-1 (WROOM-1 N16R8) | Control: motores, encoders, MPU-6050, botón |
| GOOUUU ESP32-S3-CAM V1.5 (OV3660) | Visión artificial y cámara (integrar después) |

---

## Redes de alimentación

| Red | Origen | Destino |
|---|---|---|
| VBAT | Batería + | VM del TB6612, Vin+ del conversor DC/DC (LM2596) |
| GND | Batería − | GND común de todo el sistema |
| 5V | Salida del conversor DC/DC | Pin 21 (5V) de U3, pin 20 (5V) de la GOOUUU |
| 3.3V | Pines 1 y 2 (3V3) de U3 | VCC del TB6612, STBY, VCC encoders, VCC MPU-6050, botón |

> El conversor DC/DC no usa GPIO. Solo potencia.
> Los motores se alimentan de la batería a través de VM del TB6612.
> Motores de 12V: confirmar que el voltaje de la batería esté dentro del rango del motor y del TB6612 (datasheet).

---

## ESP32-S3 DevKitC-1 (U3) — Asignación final

Pin = número del símbolo U3 en el esquemático.

| Señal | GPIO | Pin U3 | Conecta a |
|---|---|---|---|
| 3V3 | - | 1 y 2 | Rail 3.3V |
| 5V | - | 21 | Salida del DC/DC |
| GND | - | 22, 23, 24, 44 | GND común |
| PWMA | GPIO9 | 15 | TB6612 PWMA (motor izquierdo) |
| AIN2 | GPIO10 | 16 | TB6612 AIN2 |
| AIN1 | GPIO11 | 17 | TB6612 AIN1 |
| BIN1 | GPIO12 | 18 | TB6612 BIN1 (motor derecho) |
| BIN2 | GPIO13 | 19 | TB6612 BIN2 |
| PWMB | GPIO14 | 20 | TB6612 PWMB |
| CH_A_Izq | GPIO18 | 11 | Encoder motor izquierdo CH_A |
| CH_B_Izq | GPIO8 | 12 | Encoder motor izquierdo CH_B |
| CH_A_Der | GPIO41 | 38 | Encoder motor derecho CH_A |
| CH_B_Der | GPIO39 | 36 | Encoder motor derecho CH_B |
| Botón Start | GPIO40 | 37 | Botón + pull-down 10kΩ a GND |
| SDA_MPU | GPIO21 | 27 | MPU-6050 SDA |
| SCL_MPU | GPIO47 | 28 | MPU-6050 SCL |
| TXD | GPIO43 | 43 | GOOUUU RX (GPIO44) — cruzado |
| RXD | GPIO44 | 42 | GOOUUU TX (GPIO43) — cruzado |

### Firmware (Arduino IDE)

```cpp
// Motor izquierdo (A)
#define PWMA      9
#define AIN1      11
#define AIN2      10
// Motor derecho (B)
#define PWMB      14
#define BIN1      12
#define BIN2      13
// Encoders
#define ENC_IZQ_A 18
#define ENC_IZQ_B 8
#define ENC_DER_A 41
#define ENC_DER_B 39
// MPU-6050 (I2C)
#define SDA_PIN   21
#define SCL_PIN   47
// Botón Start
#define BTN_START 40
// UART a la GOOUUU: Serial0 (GPIO43 TX / GPIO44 RX)
```

### Pines libres, sacados a headers

| Header | Pines U3 | GPIO |
|---|---|---|
| H4 (Auxiliar-1 ESP32-S3) | 4, 5, 6 | GPIO4, GPIO5, GPIO6 |
| H8 (Auxiliar-1.1 ESP32-S3) | 7, 8, 9, 10 | GPIO7, GPIO15, GPIO16, GPIO17 |
| H5 (Auxiliar-2 ESP32-S3) | 41, 40, 39 | GPIO1, GPIO2, GPIO42 |

### Pines que deben quedar SIN conectar

| Pin U3 | GPIO | Motivo |
|---|---|---|
| 3 | RST | Reset |
| 13 | GPIO3 | Strapping JTAG |
| 14 | GPIO46 | Strapping |
| 25, 26 | GPIO19, GPIO20 | USB D−/D+ |
| 29 | GPIO48 | LED RGB onboard (evitado a propósito) |
| 30 | GPIO45 | Strapping VDD_SPI |
| 31 | GPIO0 | Strapping BOOT |
| 32, 33, 34 | GPIO35, GPIO36, GPIO37 | PSRAM octal (N16R8) |
| 35 | GPIO38 | Posible LED RGB según versión (evitado a propósito) |

Notas:
- GPIO39, 40, 41 y 42 también son pines JTAG, pero por defecto el S3 depura por USB, así que no afecta salvo que uses JTAG por pines.
- Programar la WROOM por el **puerto USB nativo**, no por UART, porque GPIO43/44 van a la GOOUUU.
- Arduino IDE: Board = ESP32S3 Dev Module, PSRAM = OPI PSRAM, Flash = 16MB, USB CDC On Boot = Enabled.

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
| 2 | EN/RST | Reset (sin conectar o botón a GND) |
| 26 | GPIO48 | WSLED — LED RGB onboard |
| 23 | GPIO20 | USB D+ |
| 22 | GPIO19 | USB D− |
| 29, 30, 31 | GPIO35, 36, 37 | PSRAM |
| 28 | GPIO0 | BOOT |
| 27 | GPIO45 | Strapping — no conectar |

### Comunicación con la ESP32 de control

| Pin GOOUUU | GPIO | Conecta a |
|---|---|---|
| 40 | TX / GPIO43 | U3 pin 42 (RXD, GPIO44) |
| 39 | RX / GPIO44 | U3 pin 43 (TXD, GPIO43) |
| 21 | GND | GND común — obligatorio |

### Pines libres de la GOOUUU (regleta, uso futuro)

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
| VCC | 3.3V | Lógica |
| VM | VBAT | Alimentación motores |
| GND | GND común | |
| STBY | 3.3V directo | Siempre habilitado |
| AIN1 | GPIO11 (U3 pin 17) | Dirección motor izquierdo |
| AIN2 | GPIO10 (U3 pin 16) | Dirección motor izquierdo |
| PWMA | GPIO9 (U3 pin 15) | Velocidad motor izquierdo |
| BIN1 | GPIO12 (U3 pin 18) | Dirección motor derecho |
| BIN2 | GPIO13 (U3 pin 19) | Dirección motor derecho |
| PWMB | GPIO14 (U3 pin 20) | Velocidad motor derecho |
| AO1 / AO2 | Motor izquierdo M+ / M− | Potencia |
| BO1 / BO2 | Motor derecho M+ / M− | Potencia |

### Pasivos del TB6612

| Ref | Valor | Ubicación |
|---|---|---|
| C1 | 100nF cerámico | Entre VM y GND, pegado al CI |
| C2 | 100nF cerámico | Entre VCC y GND, pegado al CI |

---

## Motores (25GA370 12V 100RPM con encoder Hall, 6 cables)

| Cable | Motor izquierdo (H1) | Motor derecho (H2) |
|---|---|---|
| M+ / M− | TB6612 AO1 / AO2 | TB6612 BO1 / BO2 |
| VCC encoder | 3.3V | 3.3V |
| GND encoder | GND común | GND común |
| CH_A | GPIO18 (pin 11) | GPIO41 (pin 38) |
| CH_B | GPIO8 (pin 12) | GPIO39 (pin 36) |

Notas:
- Corriente de stall **estimada** en ~1 A por motor (fichas de motores de la misma familia 25GA-370 / JGA25). No es dato de este modelo exacto.
- El encoder se alimenta a 3.3V (rango indicado en motores similares: 3.3 a 5V). **Nunca conectar 12V a los cables del encoder.**
- No confirmado si las salidas del encoder son de colector abierto. Si las señales salen ruidosas o fijas, usar `INPUT_PULLUP` en esos pines.

---

## MPU-6050 (H3)

| Pin MPU | Conecta a | Notas |
|---|---|---|
| VCC | 3.3V | |
| GND | GND común | |
| SDA | GPIO21 (U3 pin 27) | |
| SCL | GPIO47 (U3 pin 28) | |
| AD0 | GND | Dirección I2C = 0x68 |
| INT, XDA, XCL | Sin conectar | INT opcional (usar GPIO1 o GPIO2 de H5 si se necesita) |

Pull-ups: si usas módulo GY-521 ya trae pull-ups y capacitor. No los dupliques. Si usas el chip suelto: C3 = 100nF en VCC, R2 y R3 = 4.7kΩ a 3.3V en SDA y SCL.

---

## Botón Start (U4 + R1)

```
3.3V ──[Botón]──┬── GPIO40 (U3 pin 37)
                │
              [10kΩ]  R1
                │
               GND
```

GPIO40 lee `0` en reposo y `1` al presionar.

---

## Conversor DC/DC (LM2596)

| Pin | Conecta a |
|---|---|
| Vin+ | Batería + (VBAT) |
| Vin− | GND |
| Vout+ | 5V → U3 pin 21, GOOUUU pin 20 |
| Vout− | GND común |

Ajustar la salida a 5V **antes** de conectar las placas.

---

## Resumen de pasivos

| Ref | Valor | Cantidad | Función |
|---|---|---|---|
| C1 | 100nF | 1 | Desacople VM del TB6612 |
| C2 | 100nF | 1 | Desacople VCC del TB6612 |
| C3 | 100nF | 1 | Desacople MPU-6050 (solo si es chip suelto) |
| R1 | 10kΩ | 1 | Pull-down botón Start |
| R2, R3 | 4.7kΩ | 2 | Pull-ups I2C (solo si el MPU no trae) |

---

## Reglas de PCB (baquelita casera, quemado)

| Regla | Track (mm) | Clearance (mm) | Redes |
|---|---|---|---|
| Default | 0.5 | 0.5 | Encoders, I2C, UART, botón, PWM y dirección |
| Potencia 0.8 | 0.8 | 0.5 | AO1/AO2, BO1/BO2, 5V, 3V3 |
| Potencia 1.5 | 1.5 | 0.5 | VBAT y VM |
| Vías | 1.2 diámetro / 0.9 agujero | | |

Corrientes aproximadas por ancho (estimadas, IPC-2221, cobre 1 oz, +10 °C): 0.6 mm ~1.6 A, 0.8 mm ~2.0 A, 1.5 mm ~3 A, 1.8 mm ~3.7 A. Haz una prueba de quemado con líneas de 0.25 a 0.6 mm antes de la placa final.

## Notas para el layout

1. **GND unificado**: plano de tierra con relleno en el espacio libre, con conexión térmica en los pads para soldar a mano.
2. **VM directo a la batería** con pista ancha (regla Potencia 1.5).
3. **Desacoples pegados al CI** (C1, C2, C3).
4. **Encoders e I2C alejados de pistas de potencia.**
5. **Agrupar buses**: los 6 pines del TB6612 (U3 pines 15 a 20) juntos; 5V y 3V3 salen solo por el lado izquierdo de U3 (planificar esas pistas primero).
6. **Sin pistas entre pines de 2.54 mm** con clearance 0.5. Rodear los pines.
7. **Pocos jumpers y cortos** para los cruces en una sola capa.
8. **Dejar TX/RX enrutados** hacia la GOOUUU aunque aún no se conecte.
9. **GND compartido** entre U3 y GOOUUU (obligatorio para la UART).

---

## Pendiente de verificar (no depende del esquemático)

1. Que el footprint de U3 mapee cada número del símbolo al pin físico correcto de la DevKitC-1 (comparar con el pinout oficial) y el ancho entre las dos filas de headers (medir en la placa real).
2. Cómo está protegida la entrada de 5V de la DevKitC-1 si se conecta el DC/DC y el USB a la vez.
3. Voltaje de la batería frente a los motores de 12V y el TB6612.
4. Si las salidas del encoder necesitan pull-up.
5. Comprobar en protoboard: escaneo I2C (MPU en 0x68), lectura de ambos encoders, PWM y dirección de cada motor, y el botón, antes de fabricar.
