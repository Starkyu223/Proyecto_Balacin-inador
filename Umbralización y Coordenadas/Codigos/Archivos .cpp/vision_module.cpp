#include "vision_module.h"
#include "camera_module.h"
#include "text_overlay.h"
#include "img_converters.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_mutex   = NULL;
static uint8_t*          s_jpg_buf = NULL;
static size_t            s_jpg_len = 0;

void vision_module_init() {
  s_mutex = xSemaphoreCreateMutex();
}

static inline void set_px(camera_fb_t* fb, int x, int y, uint8_t value) {
  if (x < 0 || x >= (int)fb->width || y < 0 || y >= (int)fb->height) return;
  fb->buf[y * fb->width + x] = value;
}

// Dibuja una zona: linea horizontal en la fila analizada + marcas verticales
// en los bordes detectados + marca en el centro. `tick_span` distingue
// visualmente cada zona (la zona lejana usa marcas mas cortas que la cercana).
static void draw_zone(camera_fb_t* fb, const LineResult& r, int row, int tick_span) {
  for (int x = 0; x < (int)fb->width; x++) {
    set_px(fb, x, row, 255);
  }

  if (!r.line_found) return;

  for (int dy = -tick_span; dy <= tick_span; dy++) {
    set_px(fb, r.left_edge,  row + dy, 255);
    set_px(fb, r.right_edge, row + dy, 255);
  }

  int center_x = (r.left_edge + r.right_edge) / 2;
  for (int dy = -tick_span; dy <= tick_span; dy++) {
    set_px(fb, center_x, row + dy, 0);
  }
}

LineZonesResult vision_module_update(uint8_t threshold, int row_immediate, int row_mid, int row_far) {
  LineZonesResult zones = {
    {false, 0, 0, -1, -1},
    {false, 0, 0, -1, -1},
    {false, 0, 0, -1, -1}
  };

  camera_fb_t* fb = camera_module_capture();
  if (!fb) {
    return zones;
  }

  zones = line_processing_find_line_zones(fb, threshold, row_immediate, row_mid, row_far);

  int height = (int)fb->height;
  auto to_row = [&](int rfb) {
    int row = height - 1 - rfb;
    if (row < 0) row = 0;
    if (row >= height) row = height - 1;
    return row;
  };

  // Marcas mas largas para la zona cercana, mas cortas para la lejana:
  // asi se distinguen entre si aunque todo este en escala de grises.
  draw_zone(fb, zones.immediate, to_row(row_immediate), 8);
  draw_zone(fb, zones.mid,       to_row(row_mid),       5);
  draw_zone(fb, zones.far,       to_row(row_far),        2);

  // Overlay de texto en la esquina superior izquierda: offset_px y x absoluta
  // de cada zona, igual que se imprime en el Serial. Texto pequeno (scale=1).
  char line_i[20], line_m[20], line_f[20];
  snprintf(line_i, sizeof(line_i), "I:%d,%d",
           zones.immediate.line_found ? zones.immediate.offset_px : 0,
           zones.immediate.line_found ? zones.immediate.center_x  : 0);
  snprintf(line_m, sizeof(line_m), "M:%d,%d",
           zones.mid.line_found ? zones.mid.offset_px : 0,
           zones.mid.line_found ? zones.mid.center_x  : 0);
  snprintf(line_f, sizeof(line_f), "F:%d,%d",
           zones.far.line_found ? zones.far.offset_px : 0,
           zones.far.line_found ? zones.far.center_x  : 0);

  const int scale = 1;
  text_overlay_draw(fb, 2, 2,  line_i, scale);
  text_overlay_draw(fb, 2, 9,  line_m, scale);
  text_overlay_draw(fb, 2, 16, line_f, scale);

  uint8_t* jpg_buf = NULL;
  size_t   jpg_len = 0;
  if (frame2jpg(fb, 25, &jpg_buf, &jpg_len)) {  // calidad baja: frames mas livianos = stream mas fluido
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
      if (s_jpg_buf) {
        free(s_jpg_buf);
      }
      s_jpg_buf = jpg_buf;
      s_jpg_len = jpg_len;
      xSemaphoreGive(s_mutex);
    }
  }

  camera_module_release(fb);
  return zones;
}

bool vision_module_get_latest_jpeg(uint8_t** out_buf, size_t* out_len) {
  bool got_frame = false;
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
    if (s_jpg_buf) {
      *out_buf = (uint8_t*) malloc(s_jpg_len);
      if (*out_buf) {
        memcpy(*out_buf, s_jpg_buf, s_jpg_len);
        *out_len = s_jpg_len;
        got_frame = true;
      }
    }
    xSemaphoreGive(s_mutex);
  }
  return got_frame;
}
