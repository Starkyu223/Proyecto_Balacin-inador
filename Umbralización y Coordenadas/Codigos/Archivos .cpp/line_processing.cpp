#include "line_processing.h"

LineResult line_processing_find_line(camera_fb_t* fb, uint8_t threshold, int row_from_bottom) {
  LineResult result = {false, 0, 0, 0, 0, -1, -1};

  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) {
    return result;  // esta funcion solo trabaja con escala de grises
  }

  int width  = fb->width;
  int height = fb->height;

  int row = height - 1 - row_from_bottom;
  if (row < 0) row = 0;
  if (row >= height) row = height - 1;

  uint8_t* row_ptr = fb->buf + (row * width);

  long weighted_sum = 0;
  int  dark_pixels   = 0;
  int  left_edge     = -1;
  int  right_edge    = -1;

  for (int x = 0; x < width; x++) {
    if (row_ptr[x] < threshold) {  // pixel oscuro = parte de la linea
      weighted_sum += x;
      dark_pixels++;
      if (left_edge == -1) left_edge = x;
      right_edge = x;
    }
  }

  if (dark_pixels == 0) {
    return result;  // no se encontro linea en esta fila
  }

  int center_x     = weighted_sum / dark_pixels;
  int frame_center = width / 2;

  result.line_found  = true;
  result.pixel_count = dark_pixels;
  result.position     = ((center_x - frame_center) * 100) / frame_center;
  result.offset_px    = center_x - frame_center;  // 0 = centro, negativo = izquierda, positivo = derecha
  result.center_x     = center_x;
  result.left_edge    = left_edge;
  result.right_edge   = right_edge;

  return result;
}

LineZonesResult line_processing_find_line_zones(camera_fb_t* fb, uint8_t threshold,
                                                 int row_immediate, int row_mid, int row_far) {
  LineZonesResult zones;
  zones.immediate = line_processing_find_line(fb, threshold, row_immediate);
  zones.mid       = line_processing_find_line(fb, threshold, row_mid);
  zones.far       = line_processing_find_line(fb, threshold, row_far);
  return zones;
}
