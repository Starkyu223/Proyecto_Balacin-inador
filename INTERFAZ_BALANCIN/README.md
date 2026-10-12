# Balancín · Monitor PID

Interfaz web de un solo archivo (`balancin_pid.html`) que hace de intermediario entre el computador y la ESP32 del balancín. Permite ajustar Kp, Kd y Ki en tiempo real, ver las gráficas, visualizar el balancín y guardar un historial de datos por cada combinación de parámetros para exportarlo a Excel.

Está hecha para el firmware `PID_Paso4_PD.ino` (ESP32‑S3 + MPU6050 + TB6612FNG).

---

## 1. Requisitos y arranque

- **Navegador:** Google Chrome o Microsoft Edge. Usan la API Web Serial; Firefox y Safari no la soportan.
- **Instalación:** ninguna. Se abre con doble clic en `balancin_pid.html`. No necesita internet.
- **Cable:** la ESP32 conectada por USB.

Pasos:

1. Cierra el Monitor Serial del IDE de Arduino (el puerto solo lo puede usar un programa a la vez).
2. Abre `balancin_pid.html` en Chrome o Edge.
3. Deja los baudios en **115200** (el mismo de `Serial.begin`).
4. Pulsa **Conectar ESP32** y elige el puerto de la placa.
5. Cuando lleguen datos verás las lecturas y las gráficas moverse.

El botón **Simulador** genera datos ficticios del balancín con el mismo formato de texto que tu firmware. Sirve para probar la interfaz sin la placa. Con el simulador activo aparece el botón **Empujón**, que perturba el balancín para ver cómo responde el control.

> Algunas ESP32‑S3 se reinician al abrir el puerto serie. Es normal; la interfaz espera los datos.

---

## 2. Estructura de la pantalla

La barra superior tiene tres pestañas (**En vivo**, **Historial**, **Consola**), el estado de la conexión, la selección de baudios, el botón de tema claro/oscuro, **Conectar ESP32** y **Simulador**.

### Pestaña «En vivo»

**Columna izquierda**

| Panel | Qué hace |
|---|---|
| **Parámetros PID** | Campos de Kp, Kd y Ki con botones − / +, deslizador y botón *Enviar*. Junto a cada uno, `ESP: valor` muestra lo que la ESP reporta realmente. |
| **Control** | Estado (CONTROL ON/OFF), activar/desactivar control, *Fijar cero*, *Recalibrar* y el botón rojo **PARAR MOTORES**. |
| **Registro de historial** | Opciones de grabación y la corrida actual (Kp / Kd / Ki en uso). |
| **Mensajes de la ESP** | Últimos avisos del firmware (`>>>`, `!!!`, `===`). |

**Columna derecha**

- **Balancín (vista lateral):** dibujo del balancín que se inclina con el ángulo medido.
- **Lecturas:** Ángulo, Error, de/dt, u, PWM, Ts real, Datos/s, Setpoint y muestras en vivo.
- **Gráficas:** ángulo con setpoint, error, derivada del error, y u con PWM con signo.

### Pestaña «Historial»

Lista de corridas por parámetros, exportación, tabla comparativa, gráficas superpuestas y detalle de la corrida enfocada. Ver sección 5.

### Pestaña «Consola»

Registro de todo lo que se envía (naranja) y se recibe. Tiene un campo para enviar comandos manuales (`kp12`, `kd0.5`, `m`, `s`, `z`, `c`) y la opción *Mostrar telemetría* (desactivada por defecto para no saturar el registro).

---

## 3. Parámetros PID

- Puedes escribir decimales con **punto o coma** (`0.5` o `0,5`).
- Los botones **+/−** y las flechas ↑↓ del teclado cambian **0.1**. Con **Shift**, cambian 1.0.
- **Nada se envía a la ESP hasta que pulsas *Enviar* o Enter.** Salir del campo, usar +/− o mover el deslizador solo *preparan* el valor.
- Mientras un valor esté pendiente, el campo se resalta en naranja y el botón dice **Enviar ●**.
- La opción *Enviar al instante con deslizador y ±* (desactivada por defecto) hace que el deslizador y los botones +/− envíen automáticamente. Escribir en el campo siempre requiere Enter o *Enviar*.
- El valor `ESP:` junto a cada parámetro es el que la ESP imprimió en su última línea. Sirve para confirmar que el comando fue aceptado.

---

## 4. Comandos y seguridad

| Botón | Comando enviado | Efecto en el firmware |
|---|---|---|
| Activar / Desactivar control | `m` | Alterna el control (el firmware arranca desactivado) |
| PARAR MOTORES | `s` | Desactiva el control y detiene los motores |
| Fijar cero | `z` | La posición actual pasa a marcar 0° |
| Recalibrar | `c` | Recalibra el MPU6050 (balancín quieto ~2 s) |

- **Esc** envía `s` desde cualquier parte. **Espacio** también, si no estás escribiendo en un campo.
- El comando `m` es un interruptor. La interfaz sabe si el control está activo por la etiqueta `[CONTROL]` que el firmware añade a cada línea.
- Primera prueba siempre con las **ruedas en el aire**, como indica el firmware. Si el balancín pasa de 45° el firmware apaga el control solo.

---

## 5. Historial por parámetros

### Cómo se forman las corridas

Cada combinación de **Kp / Kd / Ki** es una *corrida*. La interfaz agrupa por los valores que **la ESP imprime en cada línea de telemetría**, no por lo que escribes en pantalla. Por eso solo aparece una corrida nueva cuando la ESP realmente recibió esos valores.

Ejemplo: si pones Kp = 10, Kd = 2 y Ki = 6, todo lo que se lea con esos valores se guarda en esa corrida. Al cambiar un parámetro y enviarlo, empieza otra. Si vuelves a una combinación anterior, las muestras nuevas se **agregan a la misma corrida**.

### Opciones de registro

- **Registrar datos:** activa o pausa el guardado en el historial.
- **Solo con control activo:** guarda únicamente muestras con `[CONTROL]` (por defecto activado). Evita llenar el historial con datos de cuando el balancín está solo en la mano.

Las gráficas en vivo muestran todo lo que llega, sin importar estas opciones.

### Qué muestra cada corrida

Tarjetas con los tags `Kp`, `Kd`, `Ki`, número de muestras, duración y RMSE. En la tabla **Comparativa**:

| Métrica | Significado |
|---|---|
| Muestras / Dur. | Cantidad de datos y duración activa (los huecos largos entre muestras se colapsan) |
| RMSE | Raíz del error cuadrático medio del ángulo respecto al setpoint (°) |
| \|e\| medio / máx | Error absoluto medio y máximo (°) |
| σ ángulo | Desviación estándar del ángulo (°) |
| % \|e\|<2° | Porcentaje del tiempo con error menor a 2° |
| \|u\| medio | Magnitud media de la salida de control |
| % sat. | Porcentaje de muestras con \|u\| ≥ 255 (saturación del PWM) |

Las métricas se calculan solo con muestras con control activo; si la corrida no tiene ninguna, se usan todas.

### Manejo del historial

- **Filtros** Kp / Kd / Ki sobre la lista (valor exacto).
- **Casillas:** marca las corridas que quieres comparar o exportar. Las gráficas de ángulo, error y u las superponen con un color por corrida.
- **Clic en una tarjeta:** la enfoca y muestra su detalle (nota editable y tabla con las últimas 200 filas).
- **Eliminar seleccionadas / Borrar todo:** piden confirmación.

### Dónde se guarda

El historial se guarda en el almacenamiento del navegador (localStorage), así que sobrevive al cerrar y reabrir. Ten en cuenta:

- Está ligado a **ese navegador y a esa ubicación del archivo**. Si abres el HTML desde otra carpeta u otro navegador, el historial será distinto.
- Si el navegador lo llena (≈5 MB), avisa. **Exporta a Excel** para respaldar tus datos.
- Borrar los datos del sitio en el navegador borra el historial.

---

## 6. Exportar a Excel y CSV

Botones en la pestaña Historial:

- **Excel (seleccionadas):** archivo `.xlsx` solo con las corridas marcadas.
- **Excel (todas):** todas las corridas.
- **CSV (seleccionadas):** texto separado por comas, UTF‑8.

El `.xlsx` contiene:

1. **Resumen:** una fila por corrida con Kp, Kd, Ki, estadísticas, fecha de inicio y nota.
2. **Datos:** todas las muestras con una columna *Corrida*, con filtro automático para filtrar por Kp / Kd / Ki.
3. **Una hoja por corrida**, con nombre tipo `Kp10 Kd2 Ki6`.

Columnas de datos: Muestra, Tiempo activo (s), Ángulo (°), Setpoint (°), Error (°), de/dt (°/s), u, PWM, Ts real (ms), Control ON, Kp, Kd, Ki.

> En Excel configurado en español, el CSV con comas puede abrirse en una sola columna. Usa el `.xlsx`, o importa el CSV desde *Datos → Desde texto/CSV*.

---

## 7. Visualizador del balancín

Vista lateral basada en tu estructura (cuerpo negro, placa naranja, rueda). Convenciones:

- **El frente está a la izquierda**, como en el balancín físico. El interruptor rojo queda atrás.
- **Ángulo positivo = inclinado hacia el frente (izquierda). Ángulo negativo = hacia atrás (derecha).**
- Las líneas discontinuas rojas marcan el corte de **±45°** (`CORTE_GRADOS`); la vertical punteada es el setpoint (0°).
- La rueda gira según el PWM con signo y el suelo se desplaza con ella. Con la convención del firmware, `u < 0` significa ruedas hacia adelante (cae hacia adelante → hay que ir hacia adelante), y se muestra "◀ ADELANTE".
- La barra bajo el dibujo indica la magnitud y dirección de la salida a motores.
- **Invertir giro visual** invierte la inclinación del dibujo, por si tu `INVERTIR_ANGULO` o el montaje del sensor lo requieren.
- Si no llegan datos por más de 1.5 s, el dibujo vuelve a vertical y muestra «sin datos».

---

## 8. Gráficas

- **Ventana:** 5 s, 10 s, 30 s, 60 s o 2 min.
- **Pausar gráficas:** congela la vista; los datos siguen llegando y registrándose.
- **Limpiar:** vacía el búfer en vivo (no toca el historial).
- Pasa el mouse sobre una gráfica para ver los valores exactos.
- Las gráficas del historial usan *tiempo activo*: los huecos entre momentos con control activo se colapsan para que las corridas se vean continuas.

---

## 9. Tema

El botón de la barra superior alterna entre tema **oscuro** (negro y naranja, por defecto) y **claro**. Se recuerda la elección.

---

## 10. Formato de telemetría esperado

La interfaz interpreta las líneas que imprime el firmware cada 200 ms:

```
Angulo: -1.23 | Error: 1.23 | de/dt: 0.0 | Kp: 10.00 | Kd: 0.00 | u: 12.3 | PWM: 0 | Ts real: 10 ms [CONTROL]
```

- Reconoce los campos por su nombre, así que el orden de los campos no importa.
- `[CONTROL]` al final indica que el control está activo.
- Las líneas que empiezan con `>>>`, `!!!` o `===` se muestran en *Mensajes de la ESP*.
- Cualquier otra línea se ve en la Consola.

**Sugerencia:** el firmware imprime a 5 Hz (`INTERVALO_MS = 200`). Para gráficas más finas y más datos por corrida, baja `INTERVALO_MS` a 50 o 20.

---

## 11. Problemas frecuentes

| Síntoma | Causa probable |
|---|---|
| No aparece la ventana para elegir puerto, o el botón no hace nada | Navegador sin Web Serial. Usa Chrome o Edge |
| Error al abrir el puerto | El Monitor Serial del IDE (u otro programa) lo tiene abierto. Ciérralo |
| Conecta pero «SIN DATOS» | Baudios distintos a `Serial.begin`, o la placa se reinició. Revisa la Consola |
| Lecturas con texto raro | Baudios incorrectos |
| `ESP:` aparece como — | Aún no ha llegado telemetría con ese parámetro |
| No se guarda el historial | El navegador bloqueó o llenó el almacenamiento. Exporta a Excel |

---

## 12. Limitaciones

- El setpoint (0°) y los demás valores de configuración del firmware no se pueden cambiar desde la interfaz; el firmware no expone comandos para ello.
- Si la ESP reporta valores fuera de formato, esa línea no se grafica (se ve en la Consola).
- Fue probada con el simulador incluido; revisa siempre el comportamiento con el balancín real, con las ruedas en el aire la primera vez.
