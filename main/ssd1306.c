#include "ssd1306.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "font5x7.h"

static const char *TAG = "oled";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ready;
static uint8_t s_fb[1 + OLED_W * OLED_H / 8]; /* leading control byte 0x40 */
#define FB (s_fb + 1)

static esp_err_t cmds(const uint8_t *c, size_t n)
{
    uint8_t buf[32];
    if (n > sizeof(buf) - 1) return ESP_ERR_INVALID_SIZE;
    buf[0] = 0x00; /* command stream */
    memcpy(buf + 1, c, n);
    return i2c_master_transmit(s_dev, buf, n + 1, 50);
}

esp_err_t ssd1306_init(void)
{
    /* Vext powers the OLED (active low), then pulse its reset line. */
    gpio_config_t io = {.pin_bit_mask = (1ULL << PIN_VEXT) | (1ULL << PIN_OLED_RST), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&io);
    gpio_set_level(PIN_VEXT, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_OLED_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_OLED_SDA,
        .scl_io_num = PIN_OLED_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bc, &s_bus);
    if (err != ESP_OK) return err;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(s_bus, &dc, &s_dev);
    if (err != ESP_OK) return err;

    static const uint8_t init[] = {
        0xae,       /* display off */
        0xd5, 0x80, /* clock divide */
        0xa8, 0x3f, /* multiplex 64 */
        0xd3, 0x00, /* display offset */
        0x40,       /* start line 0 */
        0x8d, 0x14, /* charge pump on */
        0x20, 0x00, /* horizontal addressing */
        0xa1,       /* segment remap */
        0xc8,       /* COM scan direction remapped */
        0xda, 0x12, /* COM pins */
        0x81, 0x8f, /* contrast */
        0xd9, 0xf1, /* pre-charge */
        0xdb, 0x40, /* VCOMH deselect */
        0xa4,       /* resume to RAM content */
        0xa6,       /* normal (not inverted) */
        0x2e,       /* no scrolling */
        0xaf,       /* display on */
    };
    err = cmds(init, sizeof(init));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no display found: %s", esp_err_to_name(err));
        return err;
    }
    s_fb[0] = 0x40;
    s_ready = true;
    ssd1306_clear();
    ssd1306_flush();
    return ESP_OK;
}

bool ssd1306_ready(void)
{
    return s_ready;
}

void ssd1306_clear(void)
{
    memset(FB, 0, OLED_W * OLED_H / 8);
}

static void draw_glyph(int x, int page, uint8_t c, int scale)
{
    if (c < 0x20 || c > 0x7e) c = '?';
    const uint8_t *g = FONT5X7[c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        if (scale == 1) {
            if (x + col >= 0 && x + col < OLED_W && page < 8) FB[page * OLED_W + x + col] |= bits;
        } else {
            /* Double each bit vertically across two pages, and each column horizontally. */
            uint16_t tall = 0;
            for (int b = 0; b < 8; b++) {
                if (bits & (1 << b)) tall |= (uint16_t)(3u << (2 * b));
            }
            for (int dx = 0; dx < 2; dx++) {
                int px = x + col * 2 + dx;
                if (px < 0 || px >= OLED_W) continue;
                if (page < 8) FB[page * OLED_W + px] |= (uint8_t)(tall & 0xff);
                if (page + 1 < 8) FB[(page + 1) * OLED_W + px] |= (uint8_t)(tall >> 8);
            }
        }
    }
}

int ssd1306_text(int x, int page, const char *s, int scale)
{
    if (page < 0 || page > 7 || !s) return x;
    int adv = 6 * scale;
    for (const uint8_t *p = (const uint8_t *)s; *p && x < OLED_W; p++) {
        uint8_t c = *p;
        if (c >= 0x80) {
            if ((c & 0xc0) == 0x80) continue; /* UTF-8 continuation byte */
            c = '?';
        }
        draw_glyph(x, page, c, scale);
        x += adv;
    }
    return x;
}

void ssd1306_hline(int page, int y_bit)
{
    if (page < 0 || page > 7) return;
    for (int x = 0; x < OLED_W; x++) FB[page * OLED_W + x] |= (uint8_t)(1 << y_bit);
}

void ssd1306_invert_row(int page)
{
    if (page < 0 || page > 7) return;
    for (int x = 0; x < OLED_W; x++) FB[page * OLED_W + x] ^= 0xff;
}

void ssd1306_flush(void)
{
    if (!s_ready) return;
    static const uint8_t win[] = {0x21, 0x00, 0x7f, 0x22, 0x00, 0x07};
    if (cmds(win, sizeof(win)) != ESP_OK) return;
    i2c_master_transmit(s_dev, s_fb, sizeof(s_fb), 100);
}

void ssd1306_power(bool on)
{
    if (!s_ready) return;
    uint8_t c = on ? 0xaf : 0xae;
    cmds(&c, 1);
}
