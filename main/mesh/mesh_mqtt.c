#include "mesh_mqtt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mqtt_client.h"

#include "edge_time.h"
#include "edge_util.h"

static const char *TAG = "mqtt";

typedef struct {
    char topic[MQTT_TOPIC_MAX];
    uint16_t len;
    uint8_t data[MQTT_RX_MAX];
} rx_item_t;

static esp_mqtt_client_handle_t s_client;
static QueueHandle_t s_rx;
static mesh_mqtt_cfg_t s_cfg;
static volatile bool s_connected;
static char s_last_error[64];
static uint32_t s_up, s_down, s_dropped, s_connects, s_up_fail;
static int64_t s_connected_at;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        s_connects++;
        s_connected_at = edge_uptime_ms();
        s_last_error[0] = 0;
        ESP_LOGI(TAG, "connected to %s", s_cfg.host);
        for (int i = 0; i < s_cfg.n_subs; i++) esp_mqtt_client_subscribe(s_client, s_cfg.subs[i], 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        if (s_connected) ESP_LOGW(TAG, "disconnected");
        s_connected = false;
        break;
    case MQTT_EVENT_DATA: {
        /* Messages split across events are larger than any Meshtastic packet: drop them. */
        if (ev->total_data_len != ev->data_len || ev->current_data_offset != 0 || ev->data_len > MQTT_RX_MAX ||
            ev->topic_len <= 0 || ev->topic_len >= MQTT_TOPIC_MAX || !s_rx) {
            s_dropped++;
            break;
        }
        rx_item_t *it = malloc(sizeof(rx_item_t));
        if (!it) {
            s_dropped++;
            break;
        }
        memcpy(it->topic, ev->topic, (size_t)ev->topic_len);
        it->topic[ev->topic_len] = 0;
        memcpy(it->data, ev->data, (size_t)ev->data_len);
        it->len = (uint16_t)ev->data_len;
        if (xQueueSend(s_rx, &it, 0) != pdTRUE) {
            free(it);
            s_dropped++;
        } else {
            s_down++;
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        if (ev->error_handle) {
            if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                snprintf(s_last_error, sizeof(s_last_error), "connection failed (%s)",
                         esp_err_to_name(ev->error_handle->esp_tls_last_esp_err));
            } else if (ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                snprintf(s_last_error, sizeof(s_last_error), "broker refused the login (code %d)",
                         ev->error_handle->connect_return_code);
            } else {
                snprintf(s_last_error, sizeof(s_last_error), "error %d", ev->error_handle->error_type);
            }
            ESP_LOGW(TAG, "%s", s_last_error);
        }
        break;
    default:
        break;
    }
}

esp_err_t mesh_mqtt_start(const mesh_mqtt_cfg_t *c)
{
    if (s_client) mesh_mqtt_stop();
    s_cfg = *c;
    s_rx = xQueueCreate(4, sizeof(rx_item_t *));
    if (!s_rx) return ESP_ERR_NO_MEM;
    char uri[96];
    snprintf(uri, sizeof(uri), "%s://%s:%u", c->tls ? "mqtts" : "mqtt", c->host, c->port);
    esp_mqtt_client_config_t mc = {
        .broker.address.uri = uri,
        .credentials.username = s_cfg.username[0] ? s_cfg.username : NULL,
        .credentials.authentication.password = s_cfg.password[0] ? s_cfg.password : NULL,
        .credentials.client_id = s_cfg.client_id,
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 15000,
        .network.timeout_ms = 15000,
        .buffer.size = 512,
        .buffer.out_size = 512,
        .task.stack_size = 5120,
        .task.priority = 4,
    };
    if (c->tls) mc.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) {
        vQueueDelete(s_rx);
        s_rx = NULL;
        return ESP_ERR_NO_MEM;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_event, NULL);
    s_connected = false;
    s_last_error[0] = 0;
    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) mesh_mqtt_stop();
    ESP_LOGI(TAG, "starting %s (free heap %u)", uri, (unsigned)esp_get_free_heap_size());
    return err;
}

void mesh_mqtt_stop(void)
{
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    s_connected = false;
    if (s_rx) {
        rx_item_t *it;
        while (xQueueReceive(s_rx, &it, 0) == pdTRUE) free(it);
        vQueueDelete(s_rx);
        s_rx = NULL;
    }
}

bool mesh_mqtt_running(void) { return s_client != NULL; }
bool mesh_mqtt_connected(void) { return s_connected; }

bool mesh_mqtt_publish(const char *topic, const void *data, size_t len)
{
    if (!s_client || !s_connected) return false;
    int r = esp_mqtt_client_enqueue(s_client, topic, (const char *)data, (int)len, 0, 0, true);
    if (r < 0) {
        s_up_fail++;
        return false;
    }
    s_up++;
    return true;
}

bool mesh_mqtt_take(char *topic, size_t tcap, uint8_t *buf, size_t cap, size_t *len)
{
    rx_item_t *it;
    if (!s_rx || xQueueReceive(s_rx, &it, 0) != pdTRUE) return false;
    edge_strlcpy(topic, it->topic, tcap);
    size_t n = it->len < cap ? it->len : cap;
    memcpy(buf, it->data, n);
    *len = n;
    free(it);
    return true;
}

cJSON *mesh_mqtt_status_json(void)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state", !s_client ? "off" : s_connected ? "connected" : "connecting");
    if (s_client) {
        char hp[80];
        snprintf(hp, sizeof(hp), "%s:%u", s_cfg.host, s_cfg.port);
        cJSON_AddStringToObject(o, "broker", hp);
        cJSON_AddBoolToObject(o, "tls", s_cfg.tls);
    }
    if (s_connected) cJSON_AddNumberToObject(o, "connected_for_s", (double)((edge_uptime_ms() - s_connected_at) / 1000));
    if (s_last_error[0]) cJSON_AddStringToObject(o, "last_error", s_last_error);
    cJSON_AddNumberToObject(o, "uplinked", s_up);
    cJSON_AddNumberToObject(o, "downlinked", s_down);
    cJSON_AddNumberToObject(o, "dropped", s_dropped + s_up_fail);
    cJSON_AddNumberToObject(o, "connects", s_connects);
    return o;
}
