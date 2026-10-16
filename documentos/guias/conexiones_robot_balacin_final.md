# Conexiones completas — Robot Balancín (versión final)

Este documento está alineado con el firmware **`Balacin_V1`**: los pines, el sentido de los encoders y el pulsador son los que usa ese código. Cuando algo del firmware difiere del esquemático (por ejemplo BIN1/BIN2 del motor derecho), se indica en una nota.

Cambios respecto a la v1:
- El control pasa a una **ESP32-S3-DevKitC-1 con módulo WROOM-1 N16R8** (antes el diseño estaba pensado con la distribución de la DevKitM-1 / MINI-1).
- Los números de pin de este documento son los del **símbolo U3 del esquemático**, no la posición física en la placa.
- Se reasignaron los GPIO para acortar pistas. GPIO48 y GPIO38 **no se conectan en la PCB**: son del LED RGB de la placa (según la versión) y el firmware los usa para el LED de estado.
- Se agregan headers de pines libres (H4, H8, H5).
- Reglas de PCB ajustadas para baquelita casera (quemado).

---

## Arquitectura del sistema

| MCU | Rol |
|---|---|
| ESP32-S3-DevKitC-1 (WROOM-1 N16R8) | Control: motores, encoders, MPU-6050, botón y LED RGB de estado |
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
> Los motores son de 12 V y en las pruebas trabajan con la batería a **7.5–8 V**, así que tienen menos fuerza y velocidad máxima que a 12 V (el PWM es una fracción de esa tensión).

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
| BIN1 | GPIO12 | 18 | TB6612 BIN1 (motor derecho). Ver la nota de abajo |
| BIN2 | GPIO13 | 19 | TB6612 BIN2. Ver la nota de abajo |
| PWMB | GPIO14 | 20 | TB6612 PWMB |
| CH_A_Izq | GPIO18 | 11 | Encoder motor izquierdo CH_A |
| CH_B_Izq | GPIO8 | 12 | Encoder motor izquierdo CH_B |
| CH_A_Der | GPIO41 | 38 | Encoder motor derecho CH_A |
| CH_B_Der | GPIO39 | 36 | Encoder motor derecho CH_B |
| Botón Start | GPIO40 | 37 | Botón + pull-down 10kΩ a GND (ver la sección del botón) |
| SDA_MPU | GPIO21 | 27 | MPU-6050 SDA |
| SCL_MPU | GPIO47 | 28 | MPU-6050 SCL |
| TXD | GPIO43 | 43 | GOOUUU RX (GPIO44) — cruzado |
| RXD | GPIO44 | 42 | GOOUUU TX (GPIO43) — cruzado |
| LED RGB | GPIO48 o GPIO38 | 29 o 35 | LED RGB de la placa (WS2812); no va a la PCB. El firmware escribe en los dos |

> **Nota sobre BIN1 y BIN2.** En el esquemático, GPIO12 va a BIN1 y GPIO13 a BIN2 del TB6612. En el firmware están **intercambiados** (`BIN1 = 13` y `BIN2 = 12`) para que el motor derecho gire en el sentido correcto sin tocar la PCB.

### Firmware (Arduino IDE)

Así están definidos en `Balacin_V1.ino`:

```cpp
// Motor izquierdo (A)
#define PWMA      9
#define AIN1      11
#define AIN2      10
// Motor derecho (B): BIN1/BIN2 intercambiados respecto al esquemático (ver nota)
#define PWMB      14
#define BIN1      13
#define BIN2      12
// Encoders: decodificación x4 por interrupción, con pull-up interno
#define ENC_IZQ_A 18
#define ENC_IZQ_B 8
#define ENC_DER_A 41
#define ENC_DER_B 39
#define ENC_PULLUP true
#define ENC_INVERTIR_IZQ false
#define ENC_INVERTIR_DER true       // el encoder derecho cuenta al revés
// MPU-6050 (I2C a 400 kHz, dirección 0x68)
#define SDA_PIN   21
#define SCL_PIN   47
// Pulsador de arranque
#define PIN_PULSADOR 40
#define PULSADOR_ACTIVO_BAJO true   // true = pulsador entre GPIO40 y GND
// LED RGB de la placa (WS2812): GPIO48 (placa v1.0) o GPIO38 (placa v1.1)
#define PIN_LED_RGB_A 48
#define PIN_LED_RGB_B 38
// UART a la GOOUUU: Serial0 (GPIO43 TX / GPIO44 RX). Todavía no se usa en el firmware.
```

Cuentas por vuelta de cada rueda, medidas a mano (3 vueltas, decodificación x4): **3428** la izquierda y **3342** la derecha.

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
| 29 | GPIO48 | LED RGB de la placa (placa v1.0). No se conecta en la PCB; el firmware lo usa para el LED de estado |
| 30 | GPIO45 | Strapping VDD_SPI |
| 31 | GPIO0 | Strapping BOOT |
| 32, 33, 34 | GPIO35, GPIO36, GPIO37 | PSRAM octal (N16R8) |
| 35 | GPIO38 | LED RGB de la placa (placa v1.1). No se conecta en la PCB; el firmware lo usa para el LED de estado |

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
| BIN1 | GPIO12 (U3 pin 18) | Dirección motor derecho (en el firmware es el pin `BIN2`) |
| BIN2 | GPIO13 (U3 pin 19) | Dirección motor derecho (en el firmware es el pin `BIN1`) |
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
- El firmware activa el pull-up interno (`INPUT_PULLUP`) en los cuatro pines de los encoders; sirve tanto si las salidas son de colector abierto como si no.
- Sentido de los encoders: con el comando `te` se comprobó que cada uno cuenta positivo cuando el motor gira en el sentido de la orden positiva. El derecho necesitó invertirse (`ENC_INVERTIR_DER`).
- Los motores trabajan a 7.5–8 V (ver redes de alimentación).

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

El firmware usa el bus I2C a **400 kHz** (se puede bajar con el comando `i2c100` si hay errores de lectura) y escanea el bus al arrancar: debe aparecer un dispositivo en `0x68`.

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

Con este circuito de la PCB, GPIO40 lee `0` en reposo y `1` al presionar (**activo en alto**).

**Cómo lo lee el firmware.** `PULSADOR_ACTIVO_BAJO` está en `true`: el firmware activa el pull-up interno y considera "presionado" el nivel bajo, es decir, espera un pulsador conectado **entre GPIO40 y GND**. Si se usa el botón de la PCB (3.3 V al GPIO40), hay que poner `PULSADOR_ACTIVO_BAJO` en `false`.

**Qué hace el pulsador.** Lleva el arranque en orden con tres pulsaciones: `c` (recalibrar), `z` (fijar cero) y `m` (activar el control); con el control activo, una pulsación lo para. El LED RGB indica el paso (azul, amarillo, verde y morado). El detalle está en la guía `Balacin_V1_guia.md`.

---

## LED RGB de la placa

La DevKitC-1 trae un LED RGB tipo WS2812, en el GPIO48 (placa v1.0) o en el GPIO38 (placa v1.1). No se conecta a la PCB; el firmware escribe en los dos pines con `neopixelWrite`. Colores: azul (encendido), apagado (calibrando o fijando el cero), amarillo (calibrado), verde (cero fijado), morado (control activo) y rojo (error: el MPU-6050 no responde). La placa también tiene un LED rojo fijo de alimentación, que no es este.

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

## Cómo se comprueba el conexionado

Estas comprobaciones las hace el propio firmware:

1. **MPU-6050:** al arrancar escanea el bus I2C y debe aparecer `0x68` en el Monitor Serial.
2. **Encoders y motores:** el comando `te` mueve cada motor 1 s con las ruedas en el aire y muestra las cuentas; las dos deben salir positivas.
3. **Pulsador y LED:** la secuencia de tres pulsaciones prueba el botón y el LED RGB.
4. **Motor derecho:** si gira al revés, revisa el intercambio de `BIN1`/`BIN2` descrito arriba.

## Precauciones

- No conectes al mismo tiempo el USB de la ESP32 y la salida del DC/DC (5 V) sin comprobar cómo protege la DevKitC-1 su entrada de 5 V.
- Ajusta el LM2596 a 5 V **antes** de conectar las placas.
- Nunca conectes 12 V a los cables del encoder (se alimentan a 3.3 V).
