#ifndef VISION_MODULE_H
#define VISION_MODULE_H

#include "line_processing.h"

// Inicializa recursos internos (mutex). Llamar una vez en setup(),
// despues de camera_module_init().
void vision_module_init();

// Captura un frame, detecta la linea en 3 zonas horizontales (cercana,
// media, lejana), dibuja marcas visuales distintas para cada una, lo
// convierte a JPEG y lo deja disponible para stream_module. Devuelve el
// resultado de las 3 zonas para que main.ino lo use (ej. en el control PID
// con anticipacion de curvas).
LineZonesResult vision_module_update(uint8_t threshold, int row_immediate, int row_mid, int row_far);

// Usado por stream_module: entrega una COPIA del ultimo JPEG anotado
// disponible (el llamador es responsable de hacer free() al terminar).
// Devuelve false si aun no hay ningun frame listo.
bool vision_module_get_latest_jpeg(uint8_t** out_buf, size_t* out_len);

#endif
