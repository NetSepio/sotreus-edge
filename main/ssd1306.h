// SSD1306 128x64 OLED over I2C with a local frame buffer.
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#define OLED_W 128
#define OLED_H 64

esp_err_t ssd1306_init(void);
bool ssd1306_ready(void);
void ssd1306_clear(void);
/** Draws text at pixel column x and 8-pixel page row (0..7). scale 1 or 2. Returns end x. */
int ssd1306_text(int x, int page, const char *s, int scale);
void ssd1306_hline(int page, int y_bit);
void ssd1306_invert_row(int page);
void ssd1306_flush(void);
void ssd1306_power(bool on);
