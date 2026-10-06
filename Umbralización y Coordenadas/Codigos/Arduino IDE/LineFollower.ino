#include "camera_module.h"
#include "line_processing.h"
#include "wifi_module.h"
#include "stream_module.h"
#include "vision_module.h"

// ===========================
// Datos de la red que el ESP32 va a CREAR (modo Access Point)
// ===========================
const char* AP_SSID     = "LineFollowerCam";   // nombre que vas a buscar en tu celular/PC
const char* AP_PASSWORD = "12345678";          // minimo 8 caracteres
String streamingIp = "";
unsigned long lastIpPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!camera_module_init()) {
    Serial.println("Fallo al iniciar la camara. Deteniendo.");
    while (true) delay(1000);
  }
  Serial.println("Camara lista.");

  vision_module_init();

  String ip = wifi_module_start_ap(AP_SSID, AP_PASSWORD);
  streamingIp = ip;  // se usa para reimprimir la IP periodicamente en el loop
  stream_module_start();
  Serial.printf("Streaming disponible en: http://%s:81/stream\n", ip.c_str());
}

void loop() {
  // Reimprime la IP del stream cada 5 segundos, para que siempre sea facil
  // de encontrar en el monitor sin importar cuando lo abras.
  if (millis() - lastIpPrint > 5000) {
    Serial.printf(">>> Streaming: http://%s:81/stream\n", streamingIp.c_str());
    lastIpPrint = millis();
  }

  // Filas analizadas de abajo hacia arriba del frame (QQVGA = 160x120):
  // immediate = cerca del robot, mid = zona intermedia, far = anticipacion de curvas
  LineZonesResult zones = vision_module_update(/*threshold=*/80,
                                                /*row_immediate=*/10,
                                                /*row_mid=*/50,
                                                /*row_far=*/100);

  Serial.printf(
    "Inmediata: %s offset_px=%d (x=%d) | Media: %s offset_px=%d (x=%d) | Futura: %s offset_px=%d (x=%d)\n",
    zones.immediate.line_found ? "SI" : "NO", zones.immediate.offset_px, zones.immediate.center_x,
    zones.mid.line_found       ? "SI" : "NO", zones.mid.offset_px,       zones.mid.center_x,
    zones.far.line_found       ? "SI" : "NO", zones.far.offset_px,       zones.far.center_x
  );

  // TODO: aqui se conecta con el modulo de control de motores / PID.
  // zones.immediate.position ya sirve para la correccion actual (como antes);
  // zones.mid y zones.far quedan disponibles para anticipar curvas.

  delay(50);  // ajusta segun que tan rapido necesites el ciclo de control
}
