#include "ui.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "board.h"
#include "edge_app.h"
#include "edge_battery.h"
#include "edge_ble.h"
#include "edge_config.h"
#include "edge_log.h"
#include "edge_time.h"
#include "edge_wifi.h"
#include "mesh.h"
#include "sense.h"
#include "ssd1306.h"

static const char *TAG = "ui";

#define POLL_MS         20
#define REDRAW_MS       500
#define BATTERY_MS      30000
#define PAIR_HOLD_MS    3000
#define PAIR_HOLD_MAX   8000
#define RESET_WARN_MS   5000
#define RESET_HOLD_MS   10000

enum { SCREEN_STATUS = 0, SCREEN_LINK, SCREEN_ACCESS, SCREEN_MESH, SCREEN_COUNT };

static int s_screen;
static int s_canned;            /* selected canned message on the LORA screen */
static int64_t s_tap_at;        /* pending single tap on the LORA screen (double tap = next message) */
static bool s_display_on = true;
static int64_t s_last_activity;

static void line(int page, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void line(int page, const char *fmt, ...)
{
    char buf[32];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ssd1306_text(0, page, buf, 1);
}

static void header(const char *right)
{
    char buf[32];
    snprintf(buf, sizeof(buf), " Edge %s", edge_identity()->suffix);
    ssd1306_text(0, 0, buf, 1);
    int w = (int)strlen(right) * 6;
    ssd1306_text(OLED_W - w - 2, 0, right, 1);
    ssd1306_invert_row(0);
}

static void draw_status(const edge_config_t *cfg)
{
    header(cfg->mode == EDGE_MODE_MESH ? "MESH" : "AWARE");
    if (cfg->mode == EDGE_MODE_MESH) {
        mesh_summary_t m;
        mesh_get_summary(&m);
        if (!m.radio_present) {
            line(2, "LoRa radio missing");
        } else if (!m.region_set) {
            line(2, "LoRa region not set");
            line(3, "Set it in the app or");
            line(4, "the setup page.");
        } else {
            line(2, "%s %s", m.region, m.preset ? m.preset : "");
            line(3, "%.3f MHz", m.freq_mhz);
            line(4, "Nodes %d rx %lu tx %lu", m.nodes, (unsigned long)m.rx_packets, (unsigned long)m.tx_packets);
        }
        line(5, "Awareness paused");
    } else {
        sense_stats_t s;
        sense_get_stats(&s);
        edge_log_stats_t l;
        edge_log_stats(&l);
        line(2, "BLE %-4d Wi-Fi %d", s.ble_open, s.ap_open);
        line(3, "Probes %-3d Rec %lu", s.probe_open, (unsigned long)l.count);
        line(5, l.erasing ? "Erasing records..." : "Listening");
    }
    edge_ble_info_t b;
    edge_ble_get_info(&b);
    if (b.pairing_open) {
        if (b.pairing_left_s) {
            line(6, "Pairing open %lus", (unsigned long)b.pairing_left_s);
        } else {
            line(6, "Pairing open");
        }
    } else {
        line(6, "Phone: %s", b.secured ? "connected" : b.bonds ? "paired" : "none");
    }
    edge_battery_t bat;
    edge_battery_get(&bat);
    char clock[16];
    bool have_clock = edge_time_local_hhmm(clock, sizeof(clock));
    if (bat.present) {
        /* Battery details only once a battery is actually connected. */
        line(7, "%lu.%02luV ~%d%%  %s", (unsigned long)(bat.mv / 1000), (unsigned long)(bat.mv % 1000 / 10),
             bat.percent, have_clock ? clock : "--:--");
    } else {
        line(7, "%s", have_clock ? clock : "Clock not set yet");
    }
}

static void draw_link(void)
{
    header("LINK");
    edge_wifi_info_t w;
    edge_wifi_get_info(&w);
    switch (w.state) {
    case EDGE_WIFI_AP:
        line(2, "Edge Wi-Fi network:");
        line(3, "%s", w.ssid);
        line(4, "Pass %s", edge_identity()->ap_password);
        line(5, "http://%s", w.ip);
        line(7, "%d connected", w.stations);
        break;
    case EDGE_WIFI_STA_CONNECTED:
        line(1, "Joined Wi-Fi:");
        line(2, "%s", w.ssid);
        line(4, "http://%s", w.ip);
        {
            char host[24];
            snprintf(host, sizeof(host), "sotreus-%s", edge_identity()->suffix);
            for (char *c = host; *c; c++) *c = (char)((*c >= 'A' && *c <= 'Z') ? *c + 32 : *c);
            line(5, "%s.local", host);
        }
        line(7, "Signal %d dBm", w.rssi);
        break;
    case EDGE_WIFI_STA_CONNECTING:
        line(2, "Joining Wi-Fi:");
        line(3, "%s", w.ssid);
        break;
    case EDGE_WIFI_STA_SEARCHING:
        line(2, "Looking for known");
        line(3, "Wi-Fi networks...");
        break;
    case EDGE_WIFI_STA_WAITING:
        line(2, "No known Wi-Fi");
        line(3, "in range. Retrying.");
        break;
    default:
        line(2, "Wi-Fi link off");
        line(3, "Bluetooth only");
        break;
    }
    if (w.sniffing) line(6, w.hopping ? "Listening ch 1-13" : "Listening ch %d", w.channel);
}

static void draw_access(void)
{
    header("ACCESS");
    line(1, "Setup page PIN:");
    ssd1306_text(16, 3, edge_identity()->access_pin, 2);
    line(6, "Hold PRG 3s to pair");
    line(7, "a phone over BLE.");
}

static void draw_mesh(void)
{
    header("LORA");
    mesh_summary_t m;
    mesh_get_summary(&m);
    line(1, "Node !%08lx", (unsigned long)m.node_num);
    if (!m.running) {
        line(3, "Mesh mode is off.");
        line(4, "Switch modes in the");
        line(5, "app or setup page.");
        return;
    }
    line(2, "Nodes %d  ch %.0f%%", m.nodes, m.channel_util);
    if (m.last_text[0]) {
        line(3, "From %s:", m.last_from);
        /* Wrap the last message over two lines. */
        size_t len = strlen(m.last_text);
        for (int i = 0; i < 2 && (size_t)(i * 21) < len; i++) {
            char seg[22];
            strncpy(seg, m.last_text + i * 21, 21);
            seg[21] = '\0';
            ssd1306_text(0, 4 + i, seg, 1);
        }
    } else {
        line(3, "No messages yet.");
    }
    char canned[40];
    if (mesh_canned_get(s_canned, canned, sizeof(canned))) {
        line(6, "> %s", canned);
        line(7, "2x tap next, hold send");
    }
}

static void draw_passkey(uint32_t passkey)
{
    header("PAIR");
    line(2, "Pairing code:");
    char pk[8];
    snprintf(pk, sizeof(pk), "%06lu", (unsigned long)passkey);
    ssd1306_text(28, 4, pk, 2);
    line(7, "Enter it on the phone");
}

static void draw_reset(int64_t held_ms)
{
    header("RESET");
    line(2, "Keep holding to erase");
    line(3, "settings, pairings");
    line(4, "and records.");
    char s[8];
    snprintf(s, sizeof(s), "%lds", (long)((RESET_HOLD_MS - held_ms + 999) / 1000));
    ssd1306_text(52, 5, s, 2);
}

static void wake(void)
{
    s_last_activity = edge_uptime_ms();
    if (!s_display_on) {
        ssd1306_power(true);
        s_display_on = true;
    }
}

static void ui_task(void *arg)
{
    bool have_display = false;
#if CONFIG_EDGE_OLED_ENABLE
    have_display = ssd1306_init() == ESP_OK;
#endif
    gpio_config_t btn = {.pin_bit_mask = 1ULL << PIN_BUTTON, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&btn);
    gpio_config_t led = {.pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&led);
    gpio_set_level(PIN_LED, 0);

    edge_battery_init();

    int64_t pressed_at = 0;
    bool pressed = false;
    bool reset_fired = false;
    int64_t last_redraw = 0, last_batt = edge_uptime_ms();
    s_last_activity = edge_uptime_ms();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        int64_t t = edge_uptime_ms();
        edge_config_t cfg;
        edge_config_get(&cfg);

        /* Button */
        bool down = gpio_get_level(PIN_BUTTON) == 0;
        if (down && !pressed) {
            pressed = true;
            pressed_at = t;
            reset_fired = false;
        } else if (!down && pressed) {
            pressed = false;
            int64_t held = t - pressed_at;
            if (reset_fired) {
                /* already handled */
            } else if (held >= PAIR_HOLD_MS && held <= PAIR_HOLD_MAX) {
                edge_ble_open_pairing(120);
                s_screen = SCREEN_STATUS;
                wake();
            } else if (held < 1000) {
                bool canned = s_display_on && s_screen == SCREEN_MESH && mesh_running() && mesh_canned_count() > 0;
                if (canned && s_tap_at && t - s_tap_at < 400) {
                    s_tap_at = 0; /* double tap: next canned message */
                    s_canned = (s_canned + 1) % mesh_canned_count();
                } else if (canned) {
                    s_tap_at = t; /* wait to see whether a second tap follows */
                } else if (s_display_on) {
                    s_screen = (s_screen + 1) % SCREEN_COUNT;
                }
                wake();
            } else if (held < 2500 && s_display_on && s_screen == SCREEN_MESH && mesh_running()) {
                uint32_t id;
                if (mesh_send_canned(s_canned, &id) == ESP_OK) ESP_LOGI(TAG, "canned message %d sent", s_canned);
                wake();
            } else {
                wake();
            }
            last_redraw = 0;
        }
        if (s_tap_at && t - s_tap_at >= 400) { /* single tap on the LORA screen: next screen */
            s_tap_at = 0;
            s_screen = (s_screen + 1) % SCREEN_COUNT;
            last_redraw = 0;
        }
        int64_t held = pressed ? t - pressed_at : 0;
        if (pressed && held >= RESET_HOLD_MS && !reset_fired) {
            reset_fired = true;
            ESP_LOGW(TAG, "factory reset requested from the button");
            if (have_display) {
                ssd1306_clear();
                header("RESET");
                line(3, "Erasing...");
                ssd1306_flush();
            }
            edge_app_factory_reset_later(100);
        }

        /* Battery */
        if (t - last_batt >= BATTERY_MS) {
            last_batt = t;
            edge_battery_update();
        }

        /* LED: slow blink while pairing is open. */
        static edge_ble_info_t b;
        static int64_t last_ble;
        if (t - last_ble >= 200) {
            last_ble = t;
            edge_ble_get_info(&b);
        }
        bool notify_on;
        if (mesh_led(t, &notify_on)) {
            gpio_set_level(PIN_LED, notify_on ? 1 : 0); /* external notification: a mesh message arrived */
        } else {
            gpio_set_level(PIN_LED, b.pairing_open && ((t / 500) % 2 == 0) ? 1 : 0);
        }

        if (!have_display) continue;
        if (b.passkey_active || (pressed && held >= RESET_WARN_MS)) wake();
        if (s_display_on && cfg.display_timeout_s && !b.passkey_active &&
            t - s_last_activity > (int64_t)cfg.display_timeout_s * 1000) {
            ssd1306_power(false);
            s_display_on = false;
        }
        if (!s_display_on || (t - last_redraw < REDRAW_MS && !(pressed && held >= RESET_WARN_MS))) continue;
        last_redraw = t;

        ssd1306_clear();
        if (pressed && held >= RESET_WARN_MS && !reset_fired) {
            draw_reset(held);
        } else if (b.passkey_active) {
            draw_passkey(b.passkey);
        } else {
            switch (s_screen) {
            case SCREEN_LINK: draw_link(); break;
            case SCREEN_ACCESS: draw_access(); break;
            case SCREEN_MESH: draw_mesh(); break;
            default: draw_status(&cfg); break;
            }
        }
        ssd1306_flush();
    }
}

void ui_init(void)
{
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 3, NULL, 0);
}
