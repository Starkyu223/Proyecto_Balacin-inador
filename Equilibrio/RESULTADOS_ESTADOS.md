# RESULTADOS · Equilibrio con la estructura de estados

- **Grupo:** CALABANCIN

## Ganancias (unidades del firmware)

Polos del Taller 2: [−6.0, −6.5, −1.5, −2.0] rad/s.

| Ganancia | Unidad | Valor |
|---|---|---|
| Kθ | RPM/s por grado | **64.32** |
| Kω | RPM/s por °/s | **9.31** |
| Kv | 1/s | **3.66** |
| Kx | RPM/s por vuelta | **147.49** |

## θ0

**θ0 = 2.1°** (comando `o 2.1`). En los registros, `theta` ya está medido respecto a θ0.

---

### 1. En C3, ¿cuánto se corrió el robot y qué inclinación media tuvo? ¿Cuadra con la fórmula de θ0?

**Datos de C3** (14,9 s de registro, robot en el piso sin tocarlo):

| Medida | Valor |
|---|---|
| Inclinación media θ | **−0.81°** (mín. −1.87°, máx. +0.16°, desv. 0.51°) |
| Posición x | de −0.296 a −0.252 vueltas, media −0.274 |
| Recorrido del robot | 0.044 vueltas pico a pico = **0.9 cm** (1 vuelta = 2π·0.0332 = 20.9 cm) |
| RPM deseada | oscila entre −13 y +13 RPM, media 0.1 RPM |

El robot se sostuvo casi en el mismo sitio: se movió menos de 1 cm y oscila a unos 1.2 Hz.

**¿Cuadra con la fórmula?** La fórmula de la guía es |x| = Kθ·|Δθ| / Kx, y supone que el robot está en reposo con ω = 0. Con Δθ = 0.81°:

|x| = 64.32 · 0.81 / 147.49 = 0.35 vueltas ≈ 7.4 cm

El robot **no** se corrió esos 7.4 cm, así que la fórmula sola no cuadra. La razón está en la columna `omega`: en C3 el robot está casi quieto y aun así `omega` tiene una media de **+9.95 °/s**. Si θ no cambia, la media real de la velocidad de inclinación es 0, por lo que ese valor es un **sesgo del giroscopio que no se restó bien** al arrancar. Con la ley de control completa, promediando cada término en C3:

| Término | Valor medio [RPM/s] |
|---|---|
| Kθ·θ = 64.32 · (−0.81) | −52.4 |
| Kω·ω = 9.31 · 9.95 | +92.6 |
| Kv·v = 3.66 · 0.13 | +0.5 |
| Kx·x = 147.49 · (−0.274) | −40.4 |
| **Suma** | **+0.3** |

La suma da casi 0: el término Kω·ω compensa a Kθ·θ y Kx·x. Por eso el robot no se va y a la vez θ no da 0. La fórmula de θ0 cuadra solo si ω̄ = 0; aquí el sesgo de ω (≈ 10 °/s) es lo que explica la diferencia. Corrección pendiente: volver a medir el sesgo del giroscopio con el robot completamente quieto, o restar ese valor, y repetir C3.

### 2. En C4, ¿a cuánto llegó la RPM deseada? Compárelo con la simulación del Taller 2 (sección B.4).

**Datos de C4** (12,7 s, dos empujones con un dedo):

| Empujón | RPM deseada máx. | θ máx. | x | PWM máx. |
|---|---|---|---|---|
| 1.º (t ≈ 7 s) | **−49.6 RPM** | +2.4° | de −0.29 a −0.66 (7.7 cm) | 173 |
| 2.º (t ≈ 9.6 s) | **+83.0 RPM** | −9.0° | de −0.49 a −0.01 (5.8 cm) | **255 (saturado)** |

En los dos casos el robot **se recuperó** y no se cayó. Después de cada empujón la RPM deseada volvió a cerca de 0 en unos 3 s y x quedó a menos de 2 cm del punto de reposo (≈ −0.29), dentro del criterio de 5 cm.

**Comparación con B.4.** La simulación de B.4 suelta el robot inclinado 3° con retardo de IMU de 5 ms y da:

| | Simulación B.4 | Real |
|---|---|---|
| RPM deseada máx. | **14 RPM** | 13 RPM en C3 (sin empujón) · 50 y 83 RPM en C4 |
| PWM máx. | 91 | 82 en C3 · 255 en C4 |
| |θ| máx. | 3.0° | 1.9° en C3 · 9.0° en C4 |

En reposo (C3) el robot real usa una RPM deseada parecida a la simulada (13 vs 14 RPM). En C4 la RPM deseada fue de 4 a 6 veces mayor porque el empujón es una perturbación más fuerte que los 3° iniciales de la simulación: llevó θ hasta 9° y el PWM hasta el tope de 255. Además, la simulación usa `RPM_REF_MAX = 55`, mientras que el firmware tiene `RPM_MAX = 100`; con el tope de 55 de la simulación, el segundo empujón (83 RPM) habría quedado recortado.

### 3. ¿Tuvieron que cambiar alguna ganancia? ¿Cuál, cuánto y por qué?

No se cambió ninguna ganancia Se usaron las cuatro (Kθ = 64.32, Kω = 9.31, Kv = 3.66, Kx = 147.49), 
- `SIGNO_THETA = −1`, porque `theta` salía negativo al inclinar el robot hacia adelante (paso B1). Es un signo de la medida, no de las ganancias.
- `θ0 = 2.1°`, medido en B4/B5.
- `RPM_MAX = 100` como tope de la RPM deseada.

