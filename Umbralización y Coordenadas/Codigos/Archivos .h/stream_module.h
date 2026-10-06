#ifndef STREAM_MODULE_H
#define STREAM_MODULE_H

// Inicia el servidor de streaming MJPEG en el puerto 81 (ruta /stream).
// Llamar DESPUES de camera_module_init() y wifi_module_connect().
// Corre en su propia tarea (httpd), no bloquea el loop() principal.
void stream_module_start();

#endif
