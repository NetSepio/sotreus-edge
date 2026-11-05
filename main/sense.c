#include "sense.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "edge_ble.h"
#include "edge_config.h"
#include "edge_events.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "edge_wifi.h"

static const char *TAG = "sense";

#define MAX_BLE     128
#define MAX_AP      64
#define MAX_PROBE   64
#define QUEUE_LEN   32

typedef enum { ITEM_BLE = 1, ITEM_AP, ITEM_PROBE } item_kind_t;

typedef struct {
    uint8_t kind;
    union {
        sense_ble_report_t ble;
        sense_ap_report_t ap;
        sense_probe_report_t probe;
    };
} item_t;

/* Common sighting window bookkeeping. */
typedef struct {
    bool used;
    int64_t first;
    int64_t last;
    uint32_t count;
    int8_t rssi_last, rssi_max, rssi_min;
} win_t;

typedef struct {
    win_t w;
    edge_body_ble_t b;
} ble_ent_t;

typedef struct {
    win_t w;
    edge_body_wifi_ap_t b;
} ap_ent_t;

typedef struct {
    win_t w;
    edge_body_wifi_probe_t b;
} probe_ent_t;

static QueueHandle_t s_q;
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static volatile bool s_running;
static ble_ent_t *s_ble;
static ap_ent_t *s_ap;
static probe_ent_t *s_probe;
static edge_config_t s_cfg;
static sense_stats_t s_st;

static struct {
    uint8_t kind;
    uint8_t addr[6];
    int64_t until;
    int64_t last_sent;
} s_watch;

/* ---- Feeders ---------------------------------------------------------------- */

static void enqueue(const item_t *it)
{
    if (!s_running || !s_q) return;
    if (xQueueSend(s_q, it, 0) != pdTRUE) s_st.queue_drops++;
}

void sense_feed_ble(const sense_ble_report_t *r)
{
    item_t it = {.kind = ITEM_BLE};
    it.ble = *r;
    enqueue(&it);
}

void sense_feed_ap(const sense_ap_report_t *r)
{
    item_t it = {.kind = ITEM_AP};
    it.ap = *r;
    enqueue(&it);
}

void sense_feed_probe(const sense_probe_report_t *r)
{
    item_t it = {.kind = ITEM_PROBE};
    it.probe = *r;
    enqueue(&it);
}

/* ---- Window helpers --------------------------------------------------------- */

static void win_open(win_t *w, int64_t t, int8_t rssi)
{
    w->used = true;
    w->first = w->last = t;
    w->count = 1;
    w->rssi_last = w->rssi_max = w->rssi_min = rssi;
}

static void win_hit(win_t *w, int64_t t, int8_t rssi)
{
    w->last = t;
    w->count++;
    w->rssi_last = rssi;
    if (rssi > w->rssi_max) w->rssi_max = rssi;
    if (rssi < w->rssi_min) w->rssi_min = rssi;
}

static void write_record(uint8_t type, const win_t *w, const void *body, size_t body_len, uint8_t flags)
{
    edge_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.type = type;
    rec.flags = flags;
    rec.first_ms = w->first;
    rec.last_ms = w->last;
    memcpy(rec.body, body, body_len);
    rec.body_len = (uint16_t)body_len;
    if (edge_log_append(&rec) == ESP_OK) {
        s_st.records++;
        edge_events_publish_record(&rec);
    }
}

static void flush_ble(ble_ent_t *e, uint8_t flags)
{
    e->b.rssi_last = e->w.rssi_last;
    e->b.rssi_max = e->w.rssi_max;
    e->b.rssi_min = e->w.rssi_min;
    e->b.count = e->w.count;
    write_record(EDGE_REC_BLE, &e->w, &e->b, sizeof(e->b), flags);
}

static void flush_ap(ap_ent_t *e, uint8_t flags)
{
    e->b.rssi_last = e->w.rssi_last;
    e->b.rssi_max = e->w.rssi_max;
    e->b.rssi_min = e->w.rssi_min;
    e->b.count = e->w.count;
    write_record(EDGE_REC_WIFI_AP, &e->w, &e->b, sizeof(e->b), flags);
}

static void flush_probe(probe_ent_t *e, uint8_t flags)
{
    e->b.rssi_last = e->w.rssi_last;
    e->b.rssi_max = e->w.rssi_max;
    e->b.rssi_min = e->w.rssi_min;
    e->b.count = e->w.count;
    write_record(EDGE_REC_WIFI_PROBE, &e->w, &e->b, sizeof(e->b), flags);
}

/* Returns a free slot, evicting (and writing) the least recently heard one if full. */
#define FIND_SLOT(arr, n, flushfn)                                                                 \
    ({                                                                                             \
        int _free = -1, _lru = 0;                                                                  \
        for (int _i = 0; _i < (n); _i++) {                                                         \
            if (!(arr)[_i].w.used) { _free = _i; break; }                                          \
            if ((arr)[_i].w.last < (arr)[_lru].w.last) _lru = _i;                                  \
        }                                                                                          \
        if (_free < 0) { flushfn(&(arr)[_lru], 0); (arr)[_lru].w.used = false; _free = _lru; }     \
        _free;                                                                                     \
    })

static void publish_seen(const char *kind, const uint8_t addr[6], int8_t rssi, const char *label)
{
    if (!edge_events_has_listeners()) return;
    cJSON *ev = cJSON_CreateObject();
    char m[18];
    edge_mac_str(addr, m);
    cJSON_AddStringToObject(ev, "ev", "seen");
    cJSON_AddStringToObject(ev, "kind", kind);
    cJSON_AddStringToObject(ev, "addr", m);
    cJSON_AddNumberToObject(ev, "rssi", rssi);
    if (label && label[0]) cJSON_AddStringToObject(ev, "label", label);
    cJSON_AddNumberToObject(ev, "uptime_ms", (double)edge_uptime_ms());
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void maybe_watch(uint8_t kind, const uint8_t addr[6], int8_t rssi, uint8_t channel)
{
    if (!s_watch.kind || s_watch.kind != kind || memcmp(s_watch.addr, addr, 6) != 0) return;
    int64_t t = edge_uptime_ms();
    if (t > s_watch.until) {
        s_watch.kind = 0;
        return;
    }
    if (t - s_watch.last_sent < 250) return; /* 4 Hz */
    s_watch.last_sent = t;
    cJSON *ev = cJSON_CreateObject();
    char m[18];
    edge_mac_str(addr, m);
    cJSON_AddStringToObject(ev, "ev", "rssi");
    cJSON_AddStringToObject(ev, "kind", kind == SENSE_WATCH_BLE ? "ble" : "wifi");
    cJSON_AddStringToObject(ev, "addr", m);
    cJSON_AddNumberToObject(ev, "rssi", rssi);
    if (channel) cJSON_AddNumberToObject(ev, "channel", channel);
    cJSON_AddNumberToObject(ev, "uptime_ms", (double)t);
    int64_t u = edge_time_to_unix_ms(t);
    if (u > 0) cJSON_AddNumberToObject(ev, "unix_ms", (double)u);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

/* ---- Per-kind processing ---------------------------------------------------- */

static void on_ble(const sense_ble_report_t *r, int64_t t)
{
    s_st.ble_reports++;
    if (r->rssi < s_cfg.rssi_floor) return;
    maybe_watch(SENSE_WATCH_BLE, r->addr, r->rssi, 0);
    ble_ent_t *e = NULL;
    for (int i = 0; i < MAX_BLE; i++) {
        if (s_ble[i].w.used && s_ble[i].b.addr_type == r->addr_type && memcmp(s_ble[i].b.addr, r->addr, 6) == 0) {
            e = &s_ble[i];
            break;
        }
    }
    bool is_new = false;
    if (!e) {
        e = &s_ble[FIND_SLOT(s_ble, MAX_BLE, flush_ble)];
        memset(e, 0, sizeof(*e));
        win_open(&e->w, t, r->rssi);
        memcpy(e->b.addr, r->addr, 6);
        e->b.addr_type = r->addr_type;
        is_new = true;
    } else {
        win_hit(&e->w, t, r->rssi);
    }
    uint8_t n = r->len > 31 ? 31 : r->len;
    if (r->evt_type == 4) {
        e->b.rsp_len = n;
        memcpy(e->b.rsp, r->data, n);
        e->b.flags |= EDGE_BLEF_SCAN_RSP;
    } else {
        e->b.evt_type = r->evt_type;
        e->b.adv_len = n;
        memcpy(e->b.adv, r->data, n);
        if (r->evt_type == 0 || r->evt_type == 1) {
            e->b.flags |= EDGE_BLEF_CONNECTABLE;
        } else {
            e->b.flags &= (uint8_t)~EDGE_BLEF_CONNECTABLE;
        }
    }
    if (is_new) publish_seen("ble", r->addr, r->rssi, NULL);
}

static void on_ap(const sense_ap_report_t *r, int64_t t)
{
    s_st.wifi_frames++;
    if (r->rssi < s_cfg.rssi_floor) return;
    maybe_watch(SENSE_WATCH_WIFI, r->bssid, r->rssi, r->channel);
    ap_ent_t *e = NULL;
    for (int i = 0; i < MAX_AP; i++) {
        if (s_ap[i].w.used && memcmp(s_ap[i].b.bssid, r->bssid, 6) == 0) {
            e = &s_ap[i];
            break;
        }
    }
    bool is_new = false;
    if (!e) {
        e = &s_ap[FIND_SLOT(s_ap, MAX_AP, flush_ap)];
        memset(e, 0, sizeof(*e));
        win_open(&e->w, t, r->rssi);
        memcpy(e->b.bssid, r->bssid, 6);
        e->b.flags = r->flags;
        is_new = true;
    } else {
        win_hit(&e->w, t, r->rssi);
    }
    bool from_scan = (r->flags & EDGE_APF_FROM_SCAN) != 0;
    if (!from_scan || is_new || (e->b.flags & EDGE_APF_FROM_SCAN)) {
        /* Frame detail beats scan-list detail; keep the richest view. */
        if (!from_scan) e->b.flags &= (uint8_t)~EDGE_APF_FROM_SCAN;
        e->b.channel = r->channel;
        e->b.auth = r->auth;
        if (r->beacon_itvl) e->b.beacon_itvl = r->beacon_itvl;
        if (r->ssid_len || !e->b.ssid_len) {
            e->b.ssid_len = r->ssid_len;
            memcpy(e->b.ssid, r->ssid, r->ssid_len);
            if (r->flags & EDGE_APF_HIDDEN) {
                e->b.flags |= EDGE_APF_HIDDEN;
            } else {
                e->b.flags &= (uint8_t)~EDGE_APF_HIDDEN;
            }
        }
        if (r->flags & EDGE_APF_PROBE_RESP) e->b.flags |= EDGE_APF_PROBE_RESP;
        if (r->rid_len) {
            e->b.flags |= EDGE_APF_REMOTE_ID;
            e->b.rid_len = r->rid_len;
            memcpy(e->b.rid, r->rid, r->rid_len > sizeof(e->b.rid) ? sizeof(e->b.rid) : r->rid_len);
        }
    }
    if (is_new) {
        char label[33 * 2] = "";
        if (r->ssid_len) edge_sanitize_text(label, sizeof(label), r->ssid, r->ssid_len);
        publish_seen("wifi_ap", r->bssid, r->rssi, label);
    }
}

static void on_probe(const sense_probe_report_t *r, int64_t t)
{
    s_st.wifi_frames++;
    if (r->rssi < s_cfg.rssi_floor) return;
    maybe_watch(SENSE_WATCH_WIFI, r->mac, r->rssi, r->channel);
    probe_ent_t *e = NULL;
    for (int i = 0; i < MAX_PROBE; i++) {
        if (s_probe[i].w.used && memcmp(s_probe[i].b.mac, r->mac, 6) == 0) {
            e = &s_probe[i];
            break;
        }
    }
    bool is_new = false;
    if (!e) {
        e = &s_probe[FIND_SLOT(s_probe, MAX_PROBE, flush_probe)];
        memset(e, 0, sizeof(*e));
        win_open(&e->w, t, r->rssi);
        memcpy(e->b.mac, r->mac, 6);
        if (r->mac[0] & 0x02) e->b.flags |= EDGE_PRBF_RANDOMIZED;
        is_new = true;
    } else {
        win_hit(&e->w, t, r->rssi);
    }
    e->b.channel = r->channel;
    if (r->ssid_len == 0) {
        e->b.flags |= EDGE_PRBF_WILDCARD;
    } else {
        const edge_identity_t *id = edge_identity();
        uint32_t h = edge_fnv1a(id->probe_salt, sizeof(id->probe_salt), r->ssid, r->ssid_len);
        bool known = false;
        for (int i = 0; i < e->b.n_ssids; i++) known |= e->b.ssid_hash[i] == h;
        if (!known) {
            if (e->b.n_ssids < EDGE_PROBE_MAX_SSIDS) {
                int k = e->b.n_ssids++;
                e->b.ssid_hash[k] = h;
                if (s_cfg.probe_ssid_plain) {
                    e->b.flags |= EDGE_PRBF_PLAIN;
                    e->b.plain[k].len = r->ssid_len;
                    memcpy(e->b.plain[k].ssid, r->ssid, r->ssid_len);
                }
            } else {
                e->b.flags |= EDGE_PRBF_MORE;
            }
        }
    }
    if (is_new) publish_seen("wifi_probe", r->mac, r->rssi, NULL);
}

/* ---- Window maintenance ----------------------------------------------------- */

#define MAINTAIN(arr, n, flushfn)                                                                  \
    for (int _i = 0; _i < (n); _i++) {                                                             \
        win_t *_w = &(arr)[_i].w;                                                                  \
        if (!_w->used) continue;                                                                   \
        if (all || t - _w->last >= close_ms) {                                                     \
            flushfn(&(arr)[_i], 0);                                                                \
            _w->used = false;                                                                      \
        } else if (t - _w->first >= window_ms) {                                                   \
            flushfn(&(arr)[_i], EDGE_RECF_CONTINUES);                                              \
            _w->first = t;                                                                         \
            _w->count = 0;                                                                         \
            _w->rssi_max = _w->rssi_min = _w->rssi_last;                                           \
        }                                                                                          \
    }

static void maintain(bool all)
{
    int64_t t = edge_uptime_ms();
    int64_t close_ms = (int64_t)s_cfg.close_after_s * 1000;
    int64_t window_ms = (int64_t)s_cfg.max_window_s * 1000;
    if (s_ble) MAINTAIN(s_ble, MAX_BLE, flush_ble);
    if (s_ap) MAINTAIN(s_ap, MAX_AP, flush_ap);
    if (s_probe) MAINTAIN(s_probe, MAX_PROBE, flush_probe);
}

static int count_open(void)
{
    s_st.ble_open = s_st.ap_open = s_st.probe_open = 0;
    for (int i = 0; s_ble && i < MAX_BLE; i++) s_st.ble_open += s_ble[i].w.used;
    for (int i = 0; s_ap && i < MAX_AP; i++) s_st.ap_open += s_ap[i].w.used;
    for (int i = 0; s_probe && i < MAX_PROBE; i++) s_st.probe_open += s_probe[i].w.used;
    return s_st.ble_open + s_st.ap_open + s_st.probe_open;
}

/* ---- Task ------------------------------------------------------------------- */

#define CMD_START 1
#define CMD_STOP  2
#define CMD_RELOAD 3

static QueueHandle_t s_cmd;

static void apply_radios(void)
{
    if (s_running && s_cfg.ble_scan) {
        edge_ble_scan_start(s_cfg.ble_active);
    } else {
        edge_ble_scan_stop();
    }
    bool aps = s_running && s_cfg.wifi_scan;
    bool probes = s_running && s_cfg.wifi_probes;
    edge_wifi_sniff_set(aps, probes);
}

static void free_tables(void)
{
    free(s_ble);
    free(s_ap);
    free(s_probe);
    s_ble = NULL;
    s_ap = NULL;
    s_probe = NULL;
}

static void do_start(void)
{
    if (s_running) return;
    edge_config_get(&s_cfg);
    s_ble = calloc(MAX_BLE, sizeof(ble_ent_t));
    s_ap = calloc(MAX_AP, sizeof(ap_ent_t));
    s_probe = calloc(MAX_PROBE, sizeof(probe_ent_t));
    if (!s_ble || !s_ap || !s_probe) {
        ESP_LOGE(TAG, "not enough memory for sighting tables");
        free_tables();
        return;
    }
    xQueueReset(s_q);
    s_running = true;
    apply_radios();
    ESP_LOGI(TAG, "awareness started (ble=%d wifi_ap=%d probes=%d)", s_cfg.ble_scan, s_cfg.wifi_scan,
             s_cfg.wifi_probes);
}

static void do_stop(void)
{
    if (!s_running) return;
    s_running = false;
    apply_radios();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    maintain(true); /* write every open sighting */
    free_tables();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "awareness stopped");
}

static void sense_task(void *arg)
{
    item_t *it = malloc(sizeof(item_t));
    int64_t last_maint = 0;
    for (;;) {
        uint8_t cmd;
        while (xQueueReceive(s_cmd, &cmd, 0) == pdTRUE) {
            if (cmd == CMD_START) do_start();
            if (cmd == CMD_STOP) do_stop();
            if (cmd == CMD_RELOAD && s_running) {
                edge_config_get(&s_cfg);
                apply_radios();
            }
        }
        if (xQueueReceive(s_q, it, pdMS_TO_TICKS(250)) == pdTRUE && s_running) {
            int64_t t = edge_uptime_ms();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            switch (it->kind) {
            case ITEM_BLE: if (s_cfg.ble_scan) on_ble(&it->ble, t); break;
            case ITEM_AP: if (s_cfg.wifi_scan) on_ap(&it->ap, t); break;
            case ITEM_PROBE: if (s_cfg.wifi_probes) on_probe(&it->probe, t); break;
            }
            xSemaphoreGive(s_lock);
        }
        int64_t t = edge_uptime_ms();
        if (s_running && t - last_maint >= 1000) {
            last_maint = t;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            maintain(false);
            count_open();
            xSemaphoreGive(s_lock);
        }
    }
}

void sense_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_q = xQueueCreate(QUEUE_LEN, sizeof(item_t));
    s_cmd = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreatePinnedToCore(sense_task, "sense", 4096, NULL, 5, &s_task, 1);
}

static void send_cmd(uint8_t c)
{
    xQueueSend(s_cmd, &c, portMAX_DELAY);
}

void sense_start(void)
{
    send_cmd(CMD_START);
}

void sense_stop(void)
{
    send_cmd(CMD_STOP);
}

void sense_reload(void)
{
    send_cmd(CMD_RELOAD);
}

bool sense_running(void)
{
    return s_running;
}

void sense_get_stats(sense_stats_t *out)
{
    *out = s_st;
    out->running = s_running;
    if (!s_running) out->ble_open = out->ap_open = out->probe_open = 0;
}

esp_err_t sense_watch(sense_watch_kind_t kind, const uint8_t addr[6], uint16_t seconds)
{
    if (seconds == 0 || seconds > 600) seconds = 60;
    memcpy(s_watch.addr, addr, 6);
    s_watch.until = edge_uptime_ms() + (int64_t)seconds * 1000;
    s_watch.last_sent = 0;
    s_watch.kind = (uint8_t)kind;
    return ESP_OK;
}

void sense_unwatch(void)
{
    s_watch.kind = 0;
}
