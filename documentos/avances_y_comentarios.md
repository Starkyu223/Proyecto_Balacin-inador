# Avances y comentarios

Este documento es para dejar evidencias, comentarios y avances del desarrollo del Balacin-inador: primero la PCB y después el firmware de la ESP32-S3 de control (Balacin_V1), incluyendo los problemas que fueron saliendo en el camino.

## PCB V1 y V2

### Dónde vamos

La PCB ya está diseñada (esquemático y layout hechos en EasyEDA, están en la carpeta `PCB`). Lleva el ESP32-S3 WROOM para el control, el espacio para la GOOUUU con la cámara, el TB6612FNG para los motores, el LM2596 para bajar la batería a 5V, y los conectores para los motores, el MPU-6050 y el botón.

### Los inconvenientes

#### 1. No cupieron todas las conexiones en la cara de abajo

Son muchas conexiones y no todas se pudieron pasar por la superficie inferior de cobre. Entonces opté por hacer algunos caminos con cables por la superficie superior. En el diseño se ven como **líneas rojas**, pero no son literales: solo indican qué puntos van conectados entre sí, no que el cable tenga que ir exactamente por ahí ni con esa forma. A la hora de armar, el cable se pasa por donde sea más cómodo.

#### 2. Hubo que "puentear" la masa

Por la misma cantidad de caminos, la masa (las tierras) quedó partida en diferentes secciones separadas, y la única opción fue realizar "puentes" sobre la superficie superior. Todas las tierras tienen que quedar interconectadas, es algo obligatorio.

#### 3. Blindar las señales de los cables de arriba

Para las conexiones que van por arriba con cable pensé en usar **cable UTP trenzado**: trenzar dos cables, uno con la señal y el otro a tierra. La idea es que la tierra vaya pegada a la señal y la proteja un poco de las interferencias, porque esos cables pasan cerca de caminos de más potencia o de señales PWM que meten ruido.

No es un blindaje perfecto, solo ayuda un poco, pero es barato y fácil de hacer.

#### 4. Capacitores de acople

También agregué capacitores de acople para filtrar las señales de **VCC** y **VM** (los de 100nF del TB6612, C1 y C2 en el esquemático). Sirven para limpiar el ruido que generan los motores y que no le llegue al resto del circuito.

## PCB V3

### Por qué se rehízo

Hubo un error de MercadoLibre y nos mandaron otra ESP32-S3 distinta a la que habíamos pedido. Comprar otra ESP32 salía más caro, así que la opción más económica era quedarnos con la que llegó y adaptar la PCB a esa placa.

### Qué tocó rehacer

Como la placa cambió, no bastaba con mover un par de cosas, entonces tuve que volver a rehacer:

- el esquemático,
- la asignación de pines,
- y el layout.

### Qué mejoró

Aprovechando que había que rehacer el layout, logré una mejor posición de los componentes y con eso **todas las señales ahora van por la superficie inferior de cobre** de la PCB, que era justo el problema grande de la V1 y la V2.

Ya no hacen falta los cables por arriba para las señales. Solo se van a hacer "puentes" de un **5V** y de las **masas (GND)**, que en el layout se ven como líneas rojas.

## Comentarios finales de la PCB

La continuidad de la GND se comprobó totalmente y todo quedó perfecto. Al final no se usaron cables trenzados, debido a que se logró en la PCB V3 que todas las señales de control y potencia fueran por la superficie inferior de cobre. La cámara aún no se ha probado.

## Firmware de la ESP32-S3 de control: Balacin_V1

Con la PCB lista, empezamos con el firmware de la ESP32-S3 de control. El resultado de todo este trabajo es la **versión 1 del firmware, `Balacin_V1`**. Aquí dejo cómo se llegó hasta ahí.

### De dónde partimos

Empezamos con una **base creada por mi compañera Manuela**: un PID armado paso a paso (primero la proporcional, luego la derivada y al final la integral) que ya hacía varias cosas bien:

- leía el MPU-6050 y calculaba el ángulo con un filtro complementario,
- movía los motores con el TB6612FNG,
- tenía una conexión por WiFi con el HMI (una página web) para ver los datos y cambiar las ganancias en vivo,
- y tenía protecciones básicas (zona muerta, tolerancia, antiwindup y un corte si el robot se inclina demasiado).

Era una muy buena base, pero tenía limitaciones que con las pruebas se fueron notando: **no usaba los encoders**, y todo el código (WiFi, sensor y control) corría en un solo hilo.

### Qué fui corrigiendo, en orden

De ahí en adelante yo fui corrigiendo los errores que iban saliendo, hasta tener la versión 1:

1. **El filtro del ángulo estaba mal ajustado.** Usaba un valor fijo (0.98) pensado para 100 Hz, pero el sensor se leía mucho más rápido, así que el acelerómetro pesaba demasiado y el balancín reaccionaba tarde y caía hacia adelante. Lo cambié por un filtro que usa el tiempo real entre lecturas (`alpha = tau / (tau + dt)`), con `tau` ajustable en vivo (comando `tf`). Además, la derivada del ángulo ahora la tomo directamente del giroscopio, que no tiene retraso, y subí el I2C a 400 kHz.

2. **Faltaban los encoders.** Agregué su lectura por interrupción (decodificación x4) y la velocidad de cada rueda en rpm. En esta parte mis compañeros avanzaron mucho: midieron a mano las cuentas por vuelta de cada rueda (3428 la izquierda y 3342 la derecha) y encontraron que el encoder derecho contaba al revés, así que se invirtió su sentido. Para revisar todo esto dejé el comando `te`, que prueba cada motor con las ruedas en el aire.

3. **Un lazo de solo ángulo no se sostiene.** Cualquier error pequeño en el cero hacía que el balancín se fuera escapando poco a poco hasta que los motores se saturaban y caía. Por eso armé el **control en cascada** que nos explicó el profesor: un lazo externo de **rpm** (referencia de 0 rpm) cuya salida es el ángulo que se le pide al lazo interno de **ángulo**, cada motor con su propio lazo. Al probar ese lazo en una simulación sencilla (con parámetros supuestos, solo para comparar ideas, no es el robot real) vi que **el signo estaba al revés** y realimentaba positivamente, así que lo corregí.

4. **El WiFi interfería con el control.** Como todo corría en el mismo hilo, un envío lento por WiFi retrasaba el control y el robot recibía órdenes viejas. Esta fue una de las correcciones más importantes: separé el programa en **dos núcleos**. El **núcleo 1** corre solo el control, con un reloj exacto de 10 ms, y el **núcleo 0** se encarga del WiFi, los comandos y el HMI. Lo único que los comunica es un buffer de texto y los comandos que usan el I2C o los motores (`c`, `z`, `te`, `i2c`). Así, aunque el WiFi falle o se retrase, el control sigue igual.

5. **Un motor iba hacia adelante y el otro hacia atrás, y el robot giraba.** Cada lazo de rpm usaba solo la velocidad de su rueda, y cuando el robot empieza a girar, ese mismo giro se refuerza a sí mismo. Además las integrales de los dos motores se iban separando con el tiempo. Lo resolví haciendo que los dos lazos usen la **velocidad promedio** de las ruedas y que, si las órdenes de los dos motores salen con signos opuestos, se igualen. Cada motor sigue teniendo su propio PID y sus propias ganancias, y hoy son iguales para los dos.

6. **Diagnóstico.** Agregué cosas para saber qué estaba pasando cuando algo fallaba: un contador de lecturas fallidas del MPU, el periodo máximo del lazo y el motivo del último reinicio de la ESP32. También comandos para cambiar en vivo el PWM mínimo de cada motor (`pa`, `pb`).

7. **Pulsador y LED RGB.** La ESP32 nueva sí trae LED RGB (la anterior se quemó y no lo tenía), así que el pulsador ahora lleva el arranque **en orden** y el LED muestra el paso: azul al encender, la primera pulsación recalibra (`c`) y queda en amarillo, la segunda fija el cero (`z`) y queda en verde, y la tercera activa el control (`m`) y queda en morado. Con el control activo, una pulsación lo para.

8. **Orden y claves.** Reorganicé todo el código en **dos secciones**: la Parte 1, con el control embebido (todo lo esencial para balancear), y la Parte 2, con el HMI, la comunicación y lo secundario. Cada bloque tiene su cartel explicando qué hace. Las claves del WiFi las saqué a un archivo aparte (`secretos.h`) que no se sube a GitHub.

También fui ajustando el **HMI**: las ganancias de ángulo y de cada motor en pestañas separadas (Ángulo, Motor 1 y Motor 2) y la lectura de los encoders.

### Cosas que probé y descarté

Probé un multiplicador de fuerza por motor para igualar el motor izquierdo con el derecho, pero el balancín quedó peor y lo quité. La diferencia entre los dos motores se sigue manejando con el PWM mínimo de cada uno.

### Parámetros con los que quedó la versión 1

| Parámetro | Valor |
|---|---|
| Ángulo: Kp / Ki / Kd | 40 / 0.5 / 1.1 |
| Tolerancia | 0.2° |
| rpm (los dos motores): Kp / Ki / Kd | 0.08 / 0.0002 / 0.00001 |
| Periodo del control | 10 ms (100 Hz) |
| Cero del ángulo | -4.88° |
| PWM mínimo (izquierdo / derecho) | 19 / 19 |

Los motores trabajan a 7.5–8 V (no a 12 V), así que tienen menos fuerza y velocidad máxima de la que dice la ficha, y eso limita qué tan fuerte puede corregir el balancín.

### Qué falta por confirmar

- Probar la cámara (GOOUUU) e integrarla.

### Dónde está todo

- El firmware está en `Programacion/Esp32-s3-control firmware/Balacin_V1/`.
- Una guía para estudiar cómo funciona cada parte del código (con los diagramas) está en `documentos/guias/`.
