#ifndef LINE_PROCESSING_H
#define LINE_PROCESSING_H

#include "esp_camera.h"

struct LineResult {
  bool line_found;
  int  position;     // -100 (borde izquierdo) .. 0 (centro) .. +100 (borde derecho) - normalizado, para el PID
  int  offset_px;     // posicion en PIXELES reales respecto al centro del frame: 0 = centro,
                       // negativo = linea a la izquierda, positivo = linea a la derecha
  int  center_x;      // coordenada x absoluta (0..width-1) del centro detectado de la linea
  int  pixel_count;  // cuantos pixeles "oscuros" se detectaron en la fila analizada
  int  left_edge;    // x del primer pixel oscuro encontrado (-1 si no hay linea)
  int  right_edge;   // x del ultimo pixel oscuro encontrado (-1 si no hay linea)
};

// Resultado de analizar las 3 zonas horizontales del frame (look-ahead):
// - immediate: fila mas cercana al robot -> correccion inmediata (PID actual)
// - mid:       fila intermedia -> hacia donde va la linea en breve
// - far:       fila mas lejana/arriba del frame -> anticipacion de curvas futuras
struct LineZonesResult {
  LineResult immediate;
  LineResult mid;
  LineResult far;
};

// Umbraliza una fila horizontal del frame (debe estar en PIXFORMAT_GRAYSCALE)
// y calcula el centro de la linea respecto al centro de la imagen.
//
// threshold:        0-255. Pixeles con brillo MENOR a este valor se consideran "linea".
//                    Ajustalo segun el contraste linea/fondo que tengas.
// row_from_bottom:   que tan arriba de la ultima fila se analiza (0 = fila mas cercana
//                    al carrito). Util para mirar justo frente al robot.
LineResult line_processing_find_line(camera_fb_t* fb, uint8_t threshold, int row_from_bottom);

// Analiza las 3 zonas (cercana/media/lejana) del frame, cada una en la fila
// indicada por su respectivo row_from_bottom, y devuelve un LineResult por zona.
LineZonesResult line_processing_find_line_zones(camera_fb_t* fb, uint8_t threshold,
                                                 int row_immediate, int row_mid, int row_far);

#endif
