# Balancín PID · Registro del proceso, ajustes y modificaciones

Proyecto: balancín con ESP32-S3, MPU6050 y driver TB6612FNG (sin encoders).
Archivos:
- `PID_Paso5_PID/PID_Paso5_PID.ino`: firmware (PID completo: P + I + D).
- `balancin_pid.html`: interfaz web (WiFi/WebSocket, USB o simulador).

---

## 1. Problema inicial

> Al inclinarse hacia adelante, el balancín no compensa rápido: las ruedas giran hacia donde se inclina, pero el ángulo no se corrige y termina cayendo.

### Diagnóstico (causas encontradas en el firmware)

**a) El filtro complementario no tenía el peso que se creía.**
- `ALPHA = 0.98` es un valor pensado para 100 Hz, pero `atenderMPU()` corre en cada vuelta de `loop()` (unos 300–500 Hz, no 100).
- Con ese ritmo la constante de tiempo del filtro es de ~0.1 s en vez de ~0.5 s, así que el acelerómetro pesaba demasiado.
- Cuando el carro acelera, el acelerómetro lee una inclinación falsa (menor que la real). El robot "cree" que se endereza mientras sigue cayendo y el PID no empuja lo suficiente.

**b) La derivada llegaba con retraso.**
- `de/dt` se calculaba derivando el ángulo ya filtrado y suavizado con `ALFA_D`. Ese retraso es justo lo que no puede haber para frenar una caída.

**c) Ganancias bajas por defecto.**
- Con `Kp = 10`, un error de 5° da solo ~50 de PWM.

### Correcciones propuestas (aplicadas y luego revertidas, ver sección 5)
| Cambio | Detalle |
|---|---|
| Filtro por `dt` real | `alpha = TAU_FILTRO / (TAU_FILTRO + dt)` con `TAU_FILTRO = 0.5` s y `dt` en microsegundos (limitado a 50 ms). |
| Derivada desde el giroscopio | `de/dt = -velocidad_angular_del_giroscopio` (con el signo según `INVERTIR_ANGULO`). Sin retraso y menos ruido. |
| I2C a 400 kHz | Lectura del MPU más rápida. |

### Orden recomendado para tunear
1. `ki0`, `kd0` y subir `kp` (25–40) hasta que reaccione fuerte (el PWM debe llegar casi al máximo con 7–10° de inclinación).
2. Subir `kd` (0.5–1 para empezar) hasta que pare de oscilar.
3. `ki` al final, con valores pequeños.

Si sigue cayendo con las ruedas girando fuerte, revisar `Ts real` (debe estar cerca de 10 ms): el envío por WiFi dentro de `loop()` puede meter retrasos.

---

## 2. Botones Recalibrar y Fijar cero

| Botón | Comando | Qué hace |
|---|---|---|
| **Recalibrar** | `c` | Calibra el **giroscopio**: promedia 500 lecturas con el sensor quieto (~2 s) para obtener su offset. Apaga el control y los motores, reinicia la integral y reinicia el filtro desde el ángulo del acelerómetro. No cambia el cero. |
| **Fijar cero** | `z` | Define el **punto de equilibrio**: promedia 100 lecturas del acelerómetro y guarda el valor como `anguloCero`. Esa posición pasa a marcar 0°. Imprime `#define ANGULO_CERO x.xx` para guardarlo al reiniciar. |

Orden recomendado: **Recalibrar** con el robot quieto → sostenerlo en su punto de equilibrio y **Fijar cero** → activar el control.
Si el cero está mal, el robot cree que está inclinado estando en equilibrio y se va hacia un lado.

---

## 3. Las ganancias "se reiniciaban" al recalibrar

- El código del comando `c` **no toca** Kp, Ki ni Kd; la página tampoco pisa los valores.
- Si vuelven a `kp10 / ki0 / kd0` (y `tol0.5`), lo más probable es que **el ESP32 se esté reiniciando** y esté usando sus valores por defecto.
- Sospechoso principal: *brownout* (caída de voltaje de la fuente durante la calibración o por picos del WiFi).

Medidas propuestas (aplicadas y luego revertidas, ver sección 5):
1. Guardar Kp/Ki/Kd/Tol en la flash (NVS, `Preferences`), 2 s después del último cambio y solo con el control apagado.
2. Imprimir al arrancar `esp_reset_reason()`: 1 = encendido, 3 = software, 4 = panic, 5/6/7 = watchdog, 9 = brownout.

Cómo diagnosticar sin esos cambios: tras pulsar Recalibrar, mirar el log; si aparece `Conectando a la red…` o el mensaje `=== PID Paso 5 (PID) listo…`, el ESP se reinició. Revisar entonces la alimentación (batería baja, cables flojos, ESP32 compartiendo línea con los motores).

---

## 4. Nueva pestaña "Lectura" en `balancin_pid.html`

Se agregó una pestaña que muestra de forma gráfica todo lo que reporta la ESP32 y lo que se le envía.

### Qué manda la ESP32 (cada 200 ms, dos líneas)
```
Angulo: x | Error: x | de/dt: x | Kp: x | Ki: x | Kd: x | I: x | Tol: x | u: x | PWM: x | Ts real: x ms [CONTROL]
   MPU -> Acel(g) X: x Y: x Z: x | Giro(grados/s) X: x Y: x Z: x | Ang acel: x | Ang filtrado (sin cero): x
```
La página ignoraba la segunda línea; ahora se analiza con `parseMpu()` / `onMpu()`.

### Contenido de la pestaña
- **Barra de estado:** conexión, tramas/s, antigüedad del último dato, control ON/OFF, motores. Se atenúa si dejan de llegar datos.
- **Inclinación:** esfera con aguja (+ = adelante = izquierda, igual que el balancín animado), ángulo medido, filtrado sin cero, solo acelerómetro y la diferencia acel − filtrado (sirve para ver si el acelerómetro se contamina al acelerar).
- **Acelerómetro:** X, Y, Z en g (barras de ±1.5 g) y módulo |a| (≈ 1 g quieto).
- **Giroscopio:** X, Y, Z en °/s (barras de ±250 °/s). El eje de inclinación es Y (`INCLINA_SOBRE_EJE_Y 1`).
- **Control PID:** aporte de P, I, D y u (barras de ±255), error, de/dt y tolerancia. P, I, D se recalculan en la página con Kp/Ki/Kd; u es la reportada.
- **Motores y tiempos:** PWM con signo, sentido, `Ts real` y `Ts máx` (5 s) con colores según se acerquen a 10 ms.
- **Parámetros en la ESP:** Kp, Ki, Kd y Tol.
- **Tres gráficas** (ventana 5–60 s): acelerómetro, giroscopio, y ángulo acelerómetro vs filtrado vs medido.
- **Tablas:** trama 1 y trama 2 con su texto crudo, comandos enviados a la ESP (con descripción) y mensajes de la ESP.
- **Encoders:** tarjeta informativa. El firmware actual **no tiene encoders**; cuando se agreguen y la ESP imprima sus lecturas, se mostrarán ahí.

### Cambios de código en el HTML
- Nueva pestaña (`#tab-read`) y estilos (`.rd`, `.gbar`, `.nums`, `.kv`, etc.).
- Estado nuevo en `S`: `mpu`, `mpuSamples`, `rawTel`, `rawMpu`, `txLog`.
- `send()` registra cada comando enviado; `onLine()` separa la trama del MPU.
- Funciones nuevas: `parseMpu`, `onMpu`, `initRead`, `buildDial`, `updateRead`, `drawRead`, `describeCmd`.
- El simulador ahora también genera la línea `MPU -> ...` para poder probar la pestaña sin el robot.

Verificado con el simulador (datos fluyen, gráficas y tablas se dibujan, sin errores en consola). No se probó con la ESP32 real.

---

## 5. Estado final de los archivos

- **`PID_Paso5_PID.ino`: revertido a su versión original.** A pedido, se deshicieron todos los cambios de las secciones 1 y 3 (filtro por `dt`, derivada por giroscopio, I2C a 400 kHz, guardado en flash, motivo de reinicio). Vuelven a estar `ALPHA = 0.98`, derivada del error con `ALFA_D` e I2C a 100 kHz.
- **`balancin_pid.html`: con la pestaña Lectura** (sección 4). Funciona con el firmware original, porque solo lee las dos líneas que ya imprimía.

## 6. Pendientes / ideas

- Si la caída hacia adelante persiste, reaplicar el filtro por `dt` y la derivada por giroscopio (sección 1), que son las correcciones con más probabilidad de resolverla.
- Para encoders: definir pines, imprimir una línea `Enc -> ...` en el firmware y agregar su parser y tarjeta en la pestaña Lectura.
- Diagnosticar los reinicios revisando la alimentación y, si se quiere, reponer el mensaje con `esp_reset_reason()`.
- Antes de compartir el código: `WIFI_SSID` y `WIFI_PASS` están escritos en el `.ino`.
