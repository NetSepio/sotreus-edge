// Sotreus Edge firmware entry point.
//
// Boot order: identity/config -> record log -> event bus -> radios -> HTTP ->
// UI, then the configured mode (awareness or mesh) is started.

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "edge_api.h"
#include "edge_app.h"
#include "edge_ble.h"
#include "edge_config.h"
#include "edge_console.h"
#include "edge_events.h"
#include "edge_http.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_wifi.h"
#include "mesh.h"
#include "sense.h"
#include "ui.h"

static const char *TAG = "edge";

#define STATUS_EVENT_MS     10000
#define OTA_CONFIRM_MS      30000

static int s_mode = -1;

static const char *reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "restart";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    default: return "other";
    }
}

void edge_app_apply_mode(edge_mode_t mode)
{
    if ((int)mode == s_mode) return;
    if (mode == EDGE_MODE_MESH) {
        sense_stop(); /* writes every open sighting before pausing */
        mesh_start();
    } else {
        mesh_stop();
        sense_start();
    }
    bool first = s_mode < 0;
    s_mode = (int)mode;
    const char *name = mode == EDGE_MODE_MESH ? "mesh" : "awareness";
    ESP_LOGI(TAG, "mode: %s", name);
    if (!first) edge_record_event(EDGE_EVT_MODE, name);
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mode");
    cJSON_AddStringToObject(ev, "mode", name);
    cJSON_AddBoolToObject(ev, "awareness_paused", mode == EDGE_MODE_MESH);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void reboot_cb(void *arg)
{
    esp_restart();
}

void edge_app_reboot_later(uint32_t delay_ms)
{
    static esp_timer_handle_t t;
    if (!t) {
        esp_timer_create_args_t a = {.callback = reboot_cb, .name = "reboot"};
        esp_timer_create(&a, &t);
    }
    esp_timer_stop(t);
    esp_timer_start_once(t, (uint64_t)delay_ms * 1000);
}

static void reset_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(uintptr_t)arg));
    ESP_LOGW(TAG, "factory reset: erasing records, settings and pairings");
    sense_stop();
    mesh_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    edge_log_erase(true);
    edge_config_factory_reset();
    esp_restart();
}

void edge_app_factory_reset_later(uint32_t delay_ms)
{
    static bool started;
    if (started) return;
    started = true;
    xTaskCreate(reset_task, "reset", 4096, (void *)(uintptr_t)delay_ms, 10, NULL);
}

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "Sotreus Edge %s starting (%s)", app->version, reset_reason());

    edge_time_init();
    ESP_ERROR_CHECK(edge_config_init());
    if (edge_log_init() != ESP_OK) ESP_LOGE(TAG, "record log unavailable");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    edge_events_init();
    edge_api_init();
    sense_init();
    if (mesh_init() != ESP_OK) ESP_LOGE(TAG, "mesh task could not start");
    ESP_ERROR_CHECK(edge_wifi_init());
    if (edge_ble_init() != ESP_OK) ESP_LOGE(TAG, "Bluetooth unavailable");
    edge_http_start();
    ui_init();
    edge_console_init();

    char detail[64];
    snprintf(detail, sizeof(detail), "fw %s, %s", app->version, reset_reason());
    edge_record_event(EDGE_EVT_BOOT, detail);

    edge_config_t cfg;
    edge_config_get(&cfg);
    edge_app_apply_mode((edge_mode_t)cfg.mode);

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    bool pending = esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
                   ota_state == ESP_OTA_IMG_PENDING_VERIFY;
    int64_t started = edge_uptime_ms();
    int ticks = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_EVENT_MS));
        if (pending && edge_uptime_ms() - started > OTA_CONFIRM_MS) {
            /* Ran cleanly for a while: keep this image instead of rolling back. */
            esp_ota_mark_app_valid_cancel_rollback();
            pending = false;
            ESP_LOGI(TAG, "new firmware confirmed");
        }
        if (++ticks % 6 == 0) {
            sense_stats_t ss;
            sense_get_stats(&ss);
            edge_log_stats_t ls;
            edge_log_stats(&ls);
            ESP_LOGI(TAG, "heap %u (min %u) | open ble %d ap %d probe %d | reports ble %lu wifi %lu drops %lu | records %lu",
                     (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(), ss.ble_open,
                     ss.ap_open, ss.probe_open, (unsigned long)ss.ble_reports, (unsigned long)ss.wifi_frames,
                     (unsigned long)ss.queue_drops, (unsigned long)ls.count);
        }
        if (edge_events_has_listeners()) {
            cJSON *ev = cJSON_CreateObject();
            cJSON_AddStringToObject(ev, "ev", "status");
            cJSON_AddItemToObject(ev, "status", edge_api_status_json());
            edge_events_publish(ev);
            cJSON_Delete(ev);
        }
    }
}
