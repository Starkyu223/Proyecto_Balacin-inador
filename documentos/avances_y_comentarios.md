# Avances y comentarios

Este documento es para dejar evidencias, comentarios y avances del desarrollo de la PCB del Balacin-inador, incluyendo los problemas que fueron saliendo en el camino.

## Dónde vamos

La PCB ya está diseñada (esquemático y layout hechos en EasyEDA, están en la carpeta `PCB`). Lleva el ESP32-S3 WROOM para el control, el espacio para la GOOUUU con la cámara, el TB6612FNG para los motores, el LM2596 para bajar la batería a 5V, y los conectores para los motores, el MPU-6050 y el botón.

## Los inconvenientes

### 1. No cupieron todas las conexiones en la cara de abajo

Son muchas conexiones y no todas se pudieron pasar por la superficie inferior de cobre. Entonces opté por hacer algunos caminos con cables por la superficie superior. En el diseño se ven como **líneas rojas**, pero no son literales: solo indican qué puntos van conectados entre sí, no que el cable tenga que ir exactamente por ahí ni con esa forma. A la hora de armar, el cable se pasa por donde sea más cómodo.

### 2. Hubo que "puentear" la masa

Por la misma cantidad de caminos, la masa (las tierras) quedó partida en diferentes secciones separadas, y la única opción fue realizar "puentes" sobre la superficie superior. Todas las tierras tienen que quedar interconectadas, es algo obligatorio.

### 3. Blindar las señales de los cables de arriba

Para las conexiones que van por arriba con cable pensé en usar **cable UTP trenzado**: trenzar dos cables, uno con la señal y el otro a tierra. La idea es que la tierra vaya pegada a la señal y la proteja un poco de las interferencias, porque esos cables pasan cerca de caminos de más potencia o de señales PWM que meten ruido.

No es un blindaje perfecto, solo ayuda un poco, pero es barato y fácil de hacer.

### 4. Capacitores de acople

También agregué capacitores de acople para filtrar las señales de **VCC** y **VM** (los de 100nF del TB6612, C1 y C2 en el esquemático). Sirven para limpiar el ruido que generan los motores y que no le llegue al resto del circuito.

## Pendiente / por tener en cuenta

- Al armar, revisar que todas las tierras queden realmente conectadas entre sí (con el multímetro, en continuidad).
- Fijarse bien en que los cables trenzados de señal no queden pegados a las pistas o cables de potencia de los motores.
- La cámara (GOOUUU) todavía no se integra, pero ya quedaron enrutados los pines TX/RX para cuando toque.
