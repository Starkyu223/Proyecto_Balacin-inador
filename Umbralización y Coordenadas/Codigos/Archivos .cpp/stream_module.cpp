#include "stream_module.h"
#include "esp_http_server.h"
#include "vision_module.h"
#include <Arduino.h>

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  char part_buf[64];

  while (true) {
    uint8_t* jpg_buf = NULL;
    size_t   jpg_len = 0;

    if (!vision_module_get_latest_jpeg(&jpg_buf, &jpg_len)) {
      delay(20);  // aun no hay un frame anotado listo, reintenta pronto
      continue;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);

    free(jpg_buf);

    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

void stream_module_start() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.core_id = 0;        // dejar el nucleo 1 libre para loop() (captura/deteccion/dibujo)
  config.task_priority = 5;  // suficiente para enviar frames sin quitarle CPU a WiFi

  httpd_uri_t stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL
  };

  httpd_handle_t stream_httpd = NULL;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("[stream_module] Servidor listo en http://<IP>:81/stream");
  } else {
    Serial.println("[stream_module] Error al iniciar el servidor de streaming");
  }
}
