# RESPUESTAS 
- **Grupo:** CALABANCIN
- **Geometría usada:** m_w = 0.052 kg, r = 0.0332 m, M_b = 0.732 kg, l = 0.034 m, I_b = 1.652e-3 kg·m²
- **Polos escogidos:** [−6.0, −6.5, −1.5, −2.0] rad/s · λ del lazo de RPM = 50 ms

## Las cuatro ganancias 
| Ganancia | Unidad del firmware | **Valor** |
|---|---|---|
| **Kθ** | RPM/s por grado | **64.32** |
| **Kω** | RPM/s por °/s | **9.31** |
| **Kv** | 1/s | **3.66** |
| **Kx** | RPM/s por vuelta | **147.49** |

---

## Preguntas de análisis

### 1. ¿Cuánto vale el polo inestable de su robot con el par como entrada (paso 16) y con α como entrada? ¿Por qué el segundo es menor? ¿Qué cambia eso en el periodo de muestreo?

**R:**
- Con el par como entrada: **p = √(ABg/D) = 11.64 rad/s**
- Con α como entrada: **p = √a = 6.90 rad/s** 

El segundo es menor porque con el par como entrada las ruedas giran libres y con el lazo de RPM cerrado la rueda no se deja mover, y el robot cae como un cuerpo que gira alrededor del punto de contacto. Esa inercia respecto al contacto es C + 2rB + r²A = 5.13e-3 kg·m², y el resultado es un polo más lento

### 2. Con su geometría, ¿cuál es el mínimo de kθ? Páselo a unidades del firmware (RPM/s por grado) y compárelo con la kθ que diseñaron.

**R:**

- Mínimo: (a + kx)/b = (47.60 + 2.458)/0.3519 = **142.2 rad/s² por rad = 23.7 RPM/s por grado**.
- El diseñado es 385.9 siendo **64.3 RPM/s por grado**.

El diseño queda 2.7 veces por encima del mínimo. Con ese margen la gravedad no gana, y queda holgura para el retardo de la IMU y la zona muerta.

### 3. ¿Por qué kv y kx salen positivas? Explíquelo con lo que tiene que hacer el robot para frenar.

**R:**

Para frenar el robot primero tiene que inclinarse hacia atras y para eso las ruedas se tienen que adelantar al cuerpo

Si el robot va hacia adelante (v > 0, x > 0), α debe ser positiva: la rueda acelera hacia adelante, el cuerpo se queda atrás y se inclina hacia atrás. Esa inclinación es la que después hace que el robot frene y retroceda. Por eso el término va en el **mismo sentido** que la velocidad y la posición. En la ecuación, c₁ = a·kv y c₀ = a·kx son positivos porque los polos son reales y negativos, así que kv > 0 y kx > 0.

### 4. En la parte A, ¿qué tanto se parecieron la medida y el modelo en subida, sobrepico y error? Si hubo diferencia, ¿a qué la atribuyen? ¿Qué aportaron la prealimentación (ensayo B8 de la guía), el modelo de referencia (B7) y el λ de 30 ms (B6)?

**R:**

Comparé lo medido en el robot con lo que predice el modelo. Subida es lo que tarda la rueda en pasar de 10 % a 90 % del escalón. Sobrepico es cuánto se pasa de la RPM pedida antes de acomodarse. Error es la diferencia final con la RPM pedida.

| Ensayo, λ 50 ms  | Subida medida / modelo [ms] | Sobrepico medido / modelo [%] | Error final [RPM] |
|---|---|---|---|
| 10→30 (L y R) | 25 / 30 y 25 / 25 | 23 / 19 y 19 / 19 | ≈ 0 |
| −10→−30 (L y R) | 30 / 25 y 25 / 25 | 23 / 19 y 19 / 19 | ≈ 0 |
| 0→10 (L y R) | 15 / 25 y 25 / 20 | 28 / 19 y 19 / 19 | ≈ 0 |
| 10→−10 (L y R) | 25 / 25 y 25 / 25 | 23 / 18 y 14 / 18 | ≈ 0 |

**Qué tanto se parecieron.** Bastante, en la subida el modelo acierta casi siempre con ±5 ms de diferencia. El sobrepico del modelo es de 18–19 % y el medido va de 14 a 28 %, o sea del mismo orden. El error final es casi cero en todo, porque el integrador del controlador corrige hasta llegar a la RPM pedida.

**Por qué hubo diferencias.** 

Puede ser por alguna de estas tres cosas:

- La medición de velocidad es "a pasos": solo se ve la RPM cada cierto tiempo, y por eso la subida medida salta de 5 en 5 ms.
- La batería estaba en 7.36 V (en el Lab 1 estaba en 7.43 V), así que el motor real tiene un poco menos de fuerza que la del modelo.

El caso donde más se separaron fue 0→60 RPM (B6): el modelo dio una subida de 85 ms en el izquierdo y 45 ms en el derecho, y la medida fue 65 ms en las dos. En el izquierdo el modelo además predijo 0.97 RPM de error, porque a máxima potencia (PWM 255) su recta solo llega a 59.8 RPM y la rueda real sí llegó a 60. Ese escalón está casi en el límite del motor y ahí el modelo se queda corto

### 5. ¿Qué pasó con el criterio promediado en el escalón hacia atrás y en el que cruza el cero? ¿Por qué?

**R:**

En mis datos casi no hubo diferencia

| Escalón | Por sentido (medida) | Promediada (medida) |
|---|---|---|
| 10→−10, L | 25 ms, 23 %, −0.01 | 25 ms, 23 %, −0.01 |
| 10→−10, R | 25 ms, 14 %, 0.00 | 25 ms, 14 %, −0.01 |
| −10→−30, L | 30 ms, 23 %, 0.02 | 25 ms, 23 %, 0.02 |
| −10→−30, R | 25 ms, 19 %, 0.03 | 25 ms, 19 %, 0.01 |

El motivo es que las rectas de adelante y atrás de los motores son casi iguales. En el izquierdo la K_m atrás es −0.9 % de la de adelante, y en el derecho es +2.5 %. Los N₀ difieren en 0.12 RPM (izquierdo) y 0.34 RPM (derecho). Con el promedio, la prealimentación cambia unos 2 PWM como máximo y eso lo absorbe el integrador en pocos milisegundos

### 6. De la tabla de B.5, ¿qué juego de polos escogen y por qué? ¿Qué retardo total creen que tiene su robot (IMU, filtro y periodo)?

**R:**

Escojo **[−6.0, −6.5, −1.5, −2.0]**. Es el único que se sostiene con todos los retardos de la tabla:

| Polos | IMU 0 ms | 5 ms | 10 ms | 20 ms | 30 ms |
|---|---|---|---|---|---|
| **[−6, −6.5, −1.5, −2]** | 0.02° | 0.03° | 0.03° | 0.10° | 0.15° |
| [−8, −9, −2, −2.5] | 0.13° | 0.12° | 0.10° | 2.43° | 3.07° |
| [−10, −11, −2, −2.5] | 0.11° | 0.09° | 0.10° | 2.61° | cae |
| [−12, −13, −2, −2.5] | 0.10° | 0.08° | 1.98° | 2.55° | cae |
| [−15, −16, −2, −2.5] | 0.07° | 1.66° | 1.85° | cae | cae |

Los polos más rápidos no son mejores: oscilan de forma sostenida (más de 1°) en cuanto el retardo supera unos 10 ms

**Retardo total estimado**

- IMU con su filtro interno: ≈ 5 ms.
- Periodo del lazo de equilibrio: 5 ms, y la salida se mantiene entre muestras (≈ 2.5 ms de retardo medio).
- Ventana de 10 ms de la RPM: ≈ 5 ms de retardo medio.
- Retardo propio del motor L: 0 ms según el Lab 1.

Eso da **≈ 15 ms**. Con ese valor, el segundo juego [−8, −9, …] también se sostendría (0.10°), pero con muy poco margen: a 20 ms ya oscila 2.4°. Por eso me quedo con el primero.

### 7. En la simulación de B.4, ¿llega la RPM deseada al tope? ¿Qué inclinación inicial aguanta su diseño antes de saturar?

**R:**

Con los polos escogidos y θ inicial de 3°

- La RPM deseada llega a un máximo de **13.6 RPM**, lejos del tope de 55 RPM
- El PWM máximo es **91** de 255.
- El robot se estabiliza (θ final ≈ 0°, desviación estándar 0.02°) y el eje se desplaza 2.2 cm como máximo.

Variando la inclinación inicial en la misma simulación:

| θ inicial | RPM deseada máx. | PWM máx. | Resultado |
|---|---|---|---|
| 3° | 13.6 | 91 | se sostiene |
| 6° | 27.2 | 158 | se sostiene |
| 10° | 45.5 | 251 | se sostiene |
| 10.5° | 48.0 | **255** (satura el PWM) | se sostiene |
| 12° | **55.0** (llega al tope) | 255 | se sostiene |
| 13° | 55.0 | 255 | se sostiene |
| 13.5° | 55.0 | 255 | **cae** |

El PWM empieza a saturar alrededor de **10°**, la RPM deseada llega al tope a unos **12°**, y el diseño **aguanta hasta unos 13° de inclinación inicial**. A partir de 13.5° cae. Tomo 10° como límite prudente, porque con 10° todavía queda algo de margen en el PWM.

