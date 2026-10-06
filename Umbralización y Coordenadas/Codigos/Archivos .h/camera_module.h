#ifndef CAMERA_MODULE_H
#define CAMERA_MODULE_H

#include "esp_camera.h"

// Inicializa la camara con la configuracion del proyecto (escala de grises,
// resolucion baja para procesar rapido). Devuelve true si fue exitosa.
bool camera_module_init();

// Captura un frame de la camara.
// Devuelve un puntero al framebuffer (NULL si la captura falla).
// IMPORTANTE: siempre hay que liberar el frame con camera_module_release()
// despues de usarlo, o el driver se queda sin buffers libres.
camera_fb_t* camera_module_capture();

// Libera el framebuffer capturado.
void camera_module_release(camera_fb_t* fb);

#endif
