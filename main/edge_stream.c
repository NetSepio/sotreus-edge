#include "edge_stream.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "edge_ble.h"
#include "edge_events.h"
#include "edge_log.h"
#include "edge_time.h"

static const char *TAG = "stream";

#define PEND_MAX     32      /* 8 KB of unconfirmed records */
#define ACK_TIMEOUT  30000   /* ms without a confirmation while records wait */

typedef struct {
    edge_rec_t rec;
    bool sent;
} pend_t;

static SemaphoreHandle_t s_mx;
static TaskHandle_t s_task;
static pend_t *s_pend;
static int s_head, s_n;
static volatile bool s_active;
static volatile int s_conn = -1;
static volatile bool s_stop_req;
static const char *volatile s_stop_reason;
/* Each start gets a new generation; a stop meant for an earlier one is ignored. */
static volatile uint32_t s_gen, s_stop_gen;
static int64_t s_last_ack, s_started;
static uint32_t s_delivered, s_to_flash, s_sent, s_last_acked;
static edge_rec_t s_tx; /* copy being sent */

static void publish_state(const char *state, const char *reason)
{
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "stream");
    cJSON_AddStringToObject(ev, "state", state);
    if (reason) cJSON_AddStringToObject(ev, "reason", reason);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

/* Runs inside edge_log_append, which holds the route lock. */
static bool hook(edge_rec_t *rec)
{
    if (!s_active) return false;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (!s_active || !s_pend) {
        xSemaphoreGive(s_mx);
        return false;
    }
    if (s_n == PEND_MAX) {
        /* The phone is behind: the oldest unconfirmed record goes to flash (it is the lowest seq). */
        edge_log_write_assigned(&s_pend[s_head].rec);
        s_head = (s_head + 1) % PEND_MAX;
        s_n--;
        s_to_flash++;
    }
    pend_t *p = &s_pend[(s_head + s_n) % PEND_MAX];
    p->rec = *rec;
    p->sent = false;
    s_n++;
    xSemaphoreGive(s_mx);
    if (s_task) xTaskNotifyGive(s_task);
    return true;
}

/* Writes every unconfirmed record to flash in order and ends hand-off `gen`. */
static void do_stop(const char *reason, uint32_t gen)
{
    edge_log_route_lock();
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (gen != s_gen) {
        /* A newer stream_start already took over and flushed what this one left. */
        xSemaphoreGive(s_mx);
        edge_log_route_unlock();
        return;
    }
    bool was = s_active;
    s_active = false;
    int n = s_n;
    for (int i = 0; i < s_n; i++) edge_log_write_assigned(&s_pend[(s_head + i) % PEND_MAX].rec);
    s_to_flash += (uint32_t)n;
    s_n = s_head = 0;
    free(s_pend);
    s_pend = NULL;
    s_conn = -1;
    xSemaphoreGive(s_mx);
    edge_log_route_unlock();
    if (was) {
        ESP_LOGI(TAG, "hand-off stopped (%s): %d unconfirmed record(s) written to flash", reason, n);
        publish_state("stopped", reason);
    }
}

static void stream_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
        if (s_stop_req) {
            s_stop_req = false;
            do_stop(s_stop_reason ? s_stop_reason : "stopped", s_stop_gen);
            continue;
        }
        uint32_t gen = s_gen;
        if (!s_active) continue;
        /* Send what has not been sent yet, oldest first. */
        for (;;) {
            bool have = false;
            xSemaphoreTake(s_mx, portMAX_DELAY);
            int conn = s_conn;
            for (int i = 0; s_pend && i < s_n; i++) {
                pend_t *p = &s_pend[(s_head + i) % PEND_MAX];
                if (!p->sent) {
                    s_tx = p->rec;
                    have = true;
                    break;
                }
            }
            xSemaphoreGive(s_mx);
            if (!have || !s_active) break;
            if (edge_ble_notify_record(conn, &s_tx) != 0) {
                do_stop("Bluetooth link lost", gen);
                break;
            }
            xSemaphoreTake(s_mx, portMAX_DELAY);
            for (int i = 0; s_pend && i < s_n; i++) {
                pend_t *p = &s_pend[(s_head + i) % PEND_MAX];
                if (p->rec.seq == s_tx.seq) p->sent = true;
            }
            s_sent++;
            xSemaphoreGive(s_mx);
        }
        if (s_active && s_n && edge_uptime_ms() - s_last_ack > ACK_TIMEOUT) do_stop("no confirmation from the app", gen);
    }
}

void edge_stream_init(void)
{
    s_mx = xSemaphoreCreateMutex();
    edge_log_set_hook(hook);
    xTaskCreatePinnedToCore(stream_task, "stream", 3072, NULL, 4, &s_task, 0);
}

esp_err_t edge_stream_start(int conn, uint32_t *first_seq, uint32_t *stored_head)
{
    if (!edge_ble_records_ready(conn)) return ESP_ERR_INVALID_STATE;
    /* The route lock holds numbering still, so the boundary is exact: no record can be numbered
     * between reading first_seq and switching the route to the phone. */
    edge_log_route_lock();
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_active && s_conn != conn && !s_stop_req) {
        xSemaphoreGive(s_mx);
        edge_log_route_unlock();
        return ESP_ERR_NOT_SUPPORTED; /* one phone at a time */
    }
    if (!s_pend) s_pend = calloc(PEND_MAX, sizeof(pend_t));
    esp_err_t err = s_pend ? ESP_OK : ESP_ERR_NO_MEM;
    bool restart = false;
    int flushed = 0;
    uint32_t first = 0, stored = 0;
    if (err == ESP_OK) {
        /* A start while one runs (the app lost track of it, or a stop is still queued) restarts
         * it: what is unconfirmed goes to flash, so everything below first_seq is stored. */
        restart = s_active;
        flushed = s_n;
        for (int i = 0; i < s_n; i++) edge_log_write_assigned(&s_pend[(s_head + i) % PEND_MAX].rec);
        s_to_flash += (uint32_t)s_n;
        s_head = s_n = 0;
        s_gen++;
        s_stop_req = false;
        s_conn = conn;
        s_last_ack = s_started = edge_uptime_ms();
        s_active = true;
        first = edge_log_next_seq();
        edge_log_stats_t ls;
        edge_log_stats(&ls);
        stored = ls.head_seq;
    }
    xSemaphoreGive(s_mx);
    edge_log_route_unlock();
    if (err != ESP_OK) return err;
    if (first_seq) *first_seq = first;
    if (stored_head) *stored_head = stored;
    ESP_LOGI(TAG, "hand-off to phone %s (conn %d): live from seq %lu, stored up to %lu%s",
             restart ? "restarted" : "started", conn, (unsigned long)first, (unsigned long)stored,
             flushed ? ", earlier unconfirmed records written to flash" : "");
    publish_state("started", NULL);
    return ESP_OK;
}

void edge_stream_stop(void)
{
    if (!s_active) return;
    s_stop_gen = s_gen;
    s_stop_reason = "stopped by the app";
    s_stop_req = true;
    if (s_task) xTaskNotifyGive(s_task);
}

void edge_stream_ack(uint32_t seq)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    while (s_pend && s_n && s_pend[s_head].sent && s_pend[s_head].rec.seq <= seq) {
        s_head = (s_head + 1) % PEND_MAX;
        s_n--;
        s_delivered++;
    }
    if (seq > s_last_acked) s_last_acked = seq;
    s_last_ack = edge_uptime_ms();
    xSemaphoreGive(s_mx);
}

void edge_stream_link_down(int conn)
{
    if (!s_active || s_conn != conn) return;
    s_stop_gen = s_gen;
    s_stop_reason = "Bluetooth link closed";
    s_stop_req = true;
    if (s_task) xTaskNotifyGive(s_task);
}

bool edge_stream_active(void)
{
    return s_active;
}

cJSON *edge_stream_status_json(void)
{
    cJSON *o = cJSON_CreateObject();
    xSemaphoreTake(s_mx, portMAX_DELAY);
    cJSON_AddBoolToObject(o, "active", s_active);
    cJSON_AddNumberToObject(o, "unconfirmed", s_n);
    cJSON_AddNumberToObject(o, "delivered", s_delivered);
    cJSON_AddNumberToObject(o, "written_to_flash", s_to_flash);
    if (s_last_acked) cJSON_AddNumberToObject(o, "last_confirmed_seq", s_last_acked);
    if (s_active) cJSON_AddNumberToObject(o, "active_for_s", (double)((edge_uptime_ms() - s_started) / 1000));
    xSemaphoreGive(s_mx);
    return o;
}
