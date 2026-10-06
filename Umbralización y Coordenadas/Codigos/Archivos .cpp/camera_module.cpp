#include "camera_module.h"
#include <Arduino.h>
#include "board_config.h"  // Debe tener ya seleccionado el modelo de camara correcto

bool camera_module_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk    = XCLK_GPIO_NUM;
  config.pin_pclk    = PCLK_GPIO_NUM;
  config.pin_vsync   = VSYNC_GPIO_NUM;
  config.pin_href    = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  // Escala de grises: ideal para umbralizacion, mas liviano que JPEG.
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size    = FRAMESIZE_QQVGA;  // 160x120 - suficiente para detectar la linea y rapido de procesar
  config.fb_location   = CAMERA_FB_IN_PSRAM;
  config.fb_count      = 2;
  config.grab_mode     = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[camera_module] Error al inicializar camara: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("[camera_module] No se pudo obtener el sensor tras init");
    return false;
  }

  s->set_hmirror(s, 1);  // espejar horizontalmente (izquierda <-> derecha)
  // s->set_vflip(s, 1);  // <-- descomenta esta linea si en el futuro necesitas voltear verticalmente (arriba <-> abajo)

  return true;
}

camera_fb_t* camera_module_capture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[camera_module] Captura fallida");
  }
  return fb;
}

void camera_module_release(camera_fb_t* fb) {
  if (fb) {
    esp_camera_fb_return(fb);
  }
}
