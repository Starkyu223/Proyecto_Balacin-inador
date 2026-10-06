#ifndef TEXT_OVERLAY_H
#define TEXT_OVERLAY_H

#include "esp_camera.h"

// Dibuja texto sobre un frame en escala de grises (in-place), con un fondo
// oscuro detras para que siempre se lea bien sin importar que haya debajo.
//
// Soporta: digitos 0-9, '-', ':', espacio, y las letras I, M, F (mayusculas).
// Cualquier otro caracter se dibuja como espacio en blanco.
//
// x, y:     esquina superior izquierda donde empieza el texto.
// scale:    tamano de cada "pixel" de la fuente (1 = diminuto, 2-3 = mas legible).
void text_overlay_draw(camera_fb_t* fb, int x, int y, const char* text, int scale);

#endif
