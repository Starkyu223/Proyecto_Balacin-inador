# RESULTADOS — Motores del grupo CALABANCIN

- **Fecha:** 2026-11-08
- **Batería:** 7.43 V
- **Ensayo:** device-monitor-261108-125951.log


## Tabla resumen

| Parámetro | Unidad | Motor L | Motor R |
|---|---|---|---|
| K adelante | RPM/PWM | 0.25201 | 0.26919 |
| B adelante | RPM | 4.6486 | 2.2102 |
| Zona muerta adelante | PWM | 35 | 20 |
| K atrás | RPM/PWM | 0.24968 | 0.27588 |
| B atrás | RPM | 4.52553 | 2.55097 |
| Zona muerta atrás | PWM | 30 | 20 |
| K promedio| RPM/PWM | 0.25085 | 0.27254 |
| B promedio| RPM | 4.58706 | 2.38059 |
| Zona muerta promedio | PWM | 32 | 20 |
| τ  | s | 0.03626 | 0.02655 |
| Retardo | s | 0.0 | 0.0 |
| τ de frenado | s | 0.02123 | 0.01583 |
| Retardo de frenado | s | 0.00348 | 0.0026 |
| K incremental girando adelante | RPM/PWM | 0.26012 | 0.27068 |
| K incremental girando atrás | RPM/PWM | 0.27128 | 0.27928 |
| Cuentas por vuelta | cuentas | 3285.6 | 3290.4 |

## Preguntas de análisis

### 1. En un escalón desde parado (0 → 120), la K incremental sale menor que en un escalón con la rueda girando (60 → 120). ¿Por qué? ¿Cuál de las dos describe mejor el motor para el lazo de RPM?

**Respuesta:**

&nbsp;
Desde parado, el escalón tiene que vencer la zona muerta y la fricción estática, que se ven en el término B de la recta (RPM = K·PWM − B) 

Por eso la pendiente aparente es K − B/120, y sale menor: 

- en el motor L = 0.252 − 4.65/120 = 0.213

- en el motor R = 0.269 − 2.21/120 = 0.251 

Frente a 0.260 y 0.271 con la rueda girando. Con la rueda girando ya no hay fricción estática y B no influye.

Sirve más el de la rueda girando ya que el lazo trabaja con pequeñas variaciones al momento de estar con una RPM en marcha
&nbsp;

### 2. ¿Por qué la τ al llevar el PWM a 0 sale menor que al acelerar? ¿Qué pasaría si en lugar de PWM 0 se pusiera el driver en *standby* (STBY en bajo)?

**Respuesta:**

&nbsp;
Ya que al colocar el pwm en 0 el puente h aplica una fuerza al momento de aplicarlo lo cual frena las ruedas

Si el STBY se coloca en bajo las ruedas quedarían libres y el tiempo de frenado sería mayor
&nbsp;

### 3. Comparen la zona muerta hacia adelante y hacia atrás, y la de las dos ruedas. Cuando el balancín está casi quieto, las ruedas cambian de sentido todo el tiempo. ¿Qué efecto tiene la zona muerta en ese caso?

**Respuesta:**

&nbsp;
En el motor L la zona muerta hacia adelante es **35** la de atrás es **30**, siendo asimétrica

En el motor R la zona muerta hacia adelante y para atrás es **20**, siendo simétrica

En el caso se generarían vibraciones y oscilaciones ya que las dos ruedas reaccionan distinto
&nbsp;

### 4. Con su τ y la resolución de su encoder, ¿qué limita más la rapidez del lazo de RPM, el motor o la medición de la RPM? Calculen cuántas RPM vale una cuenta en una ventana de 10 ms.

**Respuesta:**

&nbsp;
En el motor L: 60/(3285.6*0.01) = 1.8261 RPM, siendo tao del motor es 36.26 ms
 
En el motor R: 60/(3290.4*0.01) = 1.8234 RPM, siendo tao del motor es 26.55 ms

La rapidez es limitada por el motor y la medición limita la precisión ya que su lectura tendría saltos aproximados a 1.82
&nbsp;

### 5. Si la batería baja de 8,4 V a 7,4 V, ¿cuánto cambia K según la sección 4.2? ¿Qué parte del lazo de RPM tiene que corregir eso?

**Respuesta:**

&nbsp;
Bajaría un 12% y lo corrige la parte integral del lazo RPM
&nbsp;

### 6. Con la RPM máxima de su motor (PWM 255), calculen la velocidad máxima del robot en m/s. El lazo de equilibrio necesita reserva de velocidad para recuperarse cuando el robot se inclina. ¿Qué velocidad de avance les parece razonable para la pista, y por qué?

**Respuesta:**

&nbsp;
Usando la fórmula: K*255 - B

RPM del motor L:

0.25085*255 - 4.58706 = 59.38 RPM

RPM del motor R:

0.27254*255 - 2.38059 = 67.11 RPM

Usando la fórmula: V= (RPM/60) * Pi * D, con diámetro de 67.8mm

Motor L: 59.38/60 * Pi * 67.8mm = 0.2108 m/s
Motor R: 67.11/60 * Pi * 67.8mm = 0.2382 m/s

La máxima velocidad que se puede usar será la del motor L, ya que es la menor entre las dos, la velocidad puede ser la mitad de esta unos 0.1054 m/s para que asi el balancin tenga potencia para hacer ajustes a la inclinación
