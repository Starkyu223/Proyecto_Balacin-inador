#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include <Arduino.h>

// Conecta a una red WiFi EXISTENTE (modo estacion, bloquea hasta conectar).
// Devuelve la IP asignada como texto.
String wifi_module_connect(const char* ssid, const char* password);

// Hace que el ESP32 cree SU PROPIA red WiFi (modo Access Point / SoftAP).
// ssid: nombre de la red que va a crear.
// password: minimo 8 caracteres (usa "" para dejarla abierta, sin contrasena).
// Devuelve la IP del punto de acceso (normalmente 192.168.4.1).
String wifi_module_start_ap(const char* ssid, const char* password);

#endif
