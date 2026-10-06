#include "text_overlay.h"
#include <string.h>

// Fuente diminuta de 3 columnas x 5 filas. Cada fila es un valor de 3 bits
// (bit2=izquierda, bit0=derecha). Solo se incluyen los caracteres que usamos.
struct Glyph {
  char c;
  uint8_t rows[5];
};

static const Glyph FONT[] = {
  {'0', {0b111, 0b101, 0b101, 0b101, 0b111}},
  {'1', {0b010, 0b110, 0b010, 0b010, 0b111}},
  {'2', {0b111, 0b001, 0b111, 0b100, 0b111}},
  {'3', {0b111, 0b001, 0b111, 0b001, 0b111}},
  {'4', {0b101, 0b101, 0b111, 0b001, 0b001}},
  {'5', {0b111, 0b100, 0b111, 0b001, 0b111}},
  {'6', {0b111, 0b100, 0b111, 0b101, 0b111}},
  {'7', {0b111, 0b001, 0b010, 0b010, 0b010}},
  {'8', {0b111, 0b101, 0b111, 0b101, 0b111}},
  {'9', {0b111, 0b101, 0b111, 0b001, 0b111}},
  {'-', {0b000, 0b000, 0b111, 0b000, 0b000}},
  {':', {0b000, 0b010, 0b000, 0b010, 0b000}},
  {' ', {0b000, 0b000, 0b000, 0b000, 0b000}},
  {'I', {0b111, 0b010, 0b010, 0b010, 0b111}},
  {'M', {0b101, 0b111, 0b111, 0b101, 0b101}},
  {'F', {0b111, 0b100, 0b111, 0b100, 0b100}},
  {',', {0b000, 0b000, 0b000, 0b010, 0b100}},
};
static const int FONT_COUNT = sizeof(FONT) / sizeof(FONT[0]);

static const Glyph* find_glyph(char c) {
  for (int i = 0; i < FONT_COUNT; i++) {
    if (FONT[i].c == c) return &FONT[i];
  }
  return NULL;  // caracter no soportado -> se dibuja como espacio
}

static inline void set_px(camera_fb_t* fb, int x, int y, uint8_t value) {
  if (x < 0 || x >= (int)fb->width || y < 0 || y >= (int)fb->height) return;
  fb->buf[y * fb->width + x] = value;
}

static void draw_char(camera_fb_t* fb, int x, int y, char c, int scale) {
  const Glyph* g = find_glyph(c);
  if (!g) return;  // espacio / no soportado: no dibuja nada (deja el fondo)

  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 3; col++) {
      bool on = (g->rows[row] >> (2 - col)) & 0x1;
      if (!on) continue;
      // cada "pixel" logico se dibuja como un bloque de scale x scale
      for (int dy = 0; dy < scale; dy++) {
        for (int dx = 0; dx < scale; dx++) {
          set_px(fb, x + col * scale + dx, y + row * scale + dy, 255);
        }
      }
    }
  }
}

void text_overlay_draw(camera_fb_t* fb, int x, int y, const char* text, int scale) {
  int len = strlen(text);
  int char_width = 4 * scale;  // 3 columnas de glifo + 1 columna de espacio

  // Fondo oscuro detras de todo el texto, para que siempre se lea bien
  int box_w = len * char_width;
  int box_h = 5 * scale;
  for (int dy = -1; dy <= box_h; dy++) {
    for (int dx = -1; dx <= box_w; dx++) {
      set_px(fb, x + dx, y + dy, 0);
    }
  }

  for (int i = 0; i < len; i++) {
    draw_char(fb, x + i * char_width, y, text[i], scale);
  }
}
