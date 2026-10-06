#include "wifi_module.h"
#include <WiFi.h>

String wifi_module_connect(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("[wifi_module] Conectando");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  String ip = WiFi.localIP().toString();
  Serial.printf("[wifi_module] Conectado. IP: %s\n", ip.c_str());
  return ip;
}

String wifi_module_start_ap(const char* ssid, const char* password) {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, password);

  if (!ok) {
    Serial.println("[wifi_module] Error al crear el Access Point");
    return "";
  }

  String ip = WiFi.softAPIP().toString();
  Serial.printf("[wifi_module] Access Point creado: \"%s\"\n", ssid);
  Serial.printf("[wifi_module] IP: %s\n", ip.c_str());
  return ip;
}
