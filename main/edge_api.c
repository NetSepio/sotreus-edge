#include "edge_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board.h"
#include "edge_app.h"
#include "edge_ble.h"
#include "edge_battery.h"
#include "edge_config.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "edge_wifi.h"
#include "mesh.h"
#include "mesh_pb.h"
#include "mesh_radio.h"
#include "sense.h"

static const char *TAG = "api";

typedef struct {
    uint8_t sink;
    int target;
    char *line;
} job_t;

static QueueHandle_t s_jobs;
static volatile bool s_sync_busy;

/* ---- Small JSON helpers ------------------------------------------------------- */

static const char *jstr(const cJSON *o, const char *k)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsString(v) ? v->valuestring : NULL;
}

static bool jnum(const cJSON *o, const char *k, double *out)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
    if (!cJSON_IsNumber(v)) return false;
    *out = v->valuedouble;
    return true;
}

static bool jbool(const cJSON *o, const char *k, bool *out)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
    if (!cJSON_IsBool(v)) return false;
    *out = cJSON_IsTrue(v);
    return true;
}

static cJSON *ok_resp(void)
{
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "ev", "resp");
    cJSON_AddBoolToObject(r, "ok", true);
    return r;
}

static cJSON *err_resp(const char *msg)
{
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "ev", "resp");
    cJSON_AddBoolToObject(r, "ok", false);
    cJSON_AddStringToObject(r, "error", msg);
    return r;
}

static const char *mode_name(uint8_t m)
{
    return m == EDGE_MODE_MESH ? "mesh" : "awareness";
}

static const char *link_name(uint8_t l)
{
    switch (l) {
    case EDGE_LINK_AP: return "ap";
    case EDGE_LINK_STA: return "sta";
    case EDGE_LINK_OFF: return "off";
    default: return "auto";
    }
}

static const char *rebroadcast_name(uint8_t r)
{
    switch (r) {
    case EDGE_REBROADCAST_LOCAL_ONLY: return "local_only";
    case EDGE_REBROADCAST_NONE: return "none";
    default: return "all";
    }
}

static bool parse_node(const cJSON *v, uint32_t *out)
{
    if (!v || cJSON_IsNull(v)) {
        *out = MT_BROADCAST;
        return true;
    }
    if (cJSON_IsNumber(v)) {
        *out = (uint32_t)v->valuedouble;
        return *out != 0;
    }
    if (!cJSON_IsString(v)) return false;
    const char *s = v->valuestring;
    if (strcmp(s, "broadcast") == 0 || strcmp(s, "^all") == 0) {
        *out = MT_BROADCAST;
        return true;
    }
    if (s[0] == '!') s++;
    char *end = NULL;
    unsigned long n = strtoul(s, &end, 16);
    if (!end || *end || n == 0) return false;
    *out = (uint32_t)n;
    return true;
}

/* PSK text forms: "default", "none", "simpleN" (N = 0..255), "base64:<data>". */
static bool parse_psk(const char *s, edge_mesh_channel_t *ch)
{
    if (!s) return false;
    if (strcmp(s, "default") == 0) {
        ch->psk[0] = 1;
        ch->psk_len = 1;
        return true;
    }
    if (strcmp(s, "none") == 0) {
        ch->psk_len = 0;
        return true;
    }
    if (strncmp(s, "simple", 6) == 0) {
        int n = atoi(s + 6);
        if (n < 0 || n > 255) return false;
        ch->psk[0] = (uint8_t)n;
        ch->psk_len = 1;
        return true;
    }
    if (strncmp(s, "base64:", 7) == 0) {
        uint8_t k[32];
        int n = edge_b64_decode(s + 7, k, sizeof(k));
        if (n != 1 && n != 16 && n != 32) return false;
        memcpy(ch->psk, k, n);
        ch->psk_len = (uint8_t)n;
        return true;
    }
    return false;
}

static void psk_text(const edge_mesh_channel_t *ch, char *out, size_t cap)
{
    if (ch->psk_len == 0 || (ch->psk_len == 1 && ch->psk[0] == 0)) {
        edge_strlcpy(out, "none", cap);
    } else if (ch->psk_len == 1 && ch->psk[0] == 1) {
        edge_strlcpy(out, "default", cap);
    } else if (ch->psk_len == 1) {
        snprintf(out, cap, "simple%u", ch->psk[0]);
    } else {
        char b[48];
        edge_b64_encode(ch->psk, ch->psk_len, b, sizeof(b));
        snprintf(out, cap, "base64:%s", b);
    }
}

/* ---- Info / status / config views ------------------------------------------------ */

cJSON *edge_api_info_json(void)
{
    const edge_identity_t *id = edge_identity();
    const esp_app_desc_t *app = esp_app_get_description();
    cJSON *o = cJSON_CreateObject();
    char hex[17];
    edge_hex(id->device_id, sizeof(id->device_id), hex);
    cJSON_AddNumberToObject(o, "protocol", EDGE_PROTOCOL_VERSION);
    cJSON_AddStringToObject(o, "device_id", hex);
    cJSON_AddStringToObject(o, "name", "Sotreus Edge");
    cJSON_AddStringToObject(o, "suffix", id->suffix);
    cJSON_AddStringToObject(o, "fw", app->version);
    cJSON_AddStringToObject(o, "idf", esp_get_idf_version());
    cJSON_AddStringToObject(o, "hw", EDGE_HW_MODEL);
    cJSON_AddNumberToObject(o, "boot", id->boot_count);
    cJSON_AddNumberToObject(o, "record_size", EDGE_REC_SIZE);
    cJSON_AddNumberToObject(o, "record_format", EDGE_REC_FORMAT);
    edge_hex(id->probe_salt, sizeof(id->probe_salt), hex);
    cJSON_AddStringToObject(o, "probe_salt", hex);
    cJSON *caps = cJSON_AddObjectToObject(o, "caps");
    cJSON_AddTrueToObject(caps, "ble_observation");
    cJSON_AddTrueToObject(caps, "wifi24_observation");
    cJSON_AddTrueToObject(caps, "wifi_probe_observation");
    cJSON_AddFalseToObject(caps, "gnss");
    cJSON_AddTrueToObject(caps, "local_storage");
    cJSON_AddFalseToObject(caps, "firmware_switching");
    cJSON_AddTrueToObject(caps, "meshtastic_mode");
    cJSON_AddTrueToObject(caps, "lora");
    cJSON_AddTrueToObject(caps, "display");
    cJSON_AddTrueToObject(caps, "battery");
    cJSON_AddTrueToObject(caps, "ota_upload");
    return o;
}

cJSON *edge_api_status_json(void)
{
    edge_config_t cfg;
    edge_config_get(&cfg);
    cJSON *o = cJSON_CreateObject();
    int64_t up = edge_uptime_ms();
    cJSON_AddNumberToObject(o, "uptime_ms", (double)up);
    cJSON_AddBoolToObject(o, "time_valid", edge_time_valid());
    cJSON_AddStringToObject(o, "time_source", edge_time_source());
    if (edge_time_valid()) cJSON_AddNumberToObject(o, "unix_ms", (double)edge_time_to_unix_ms(up));
    cJSON_AddStringToObject(o, "mode", mode_name(cfg.mode));

    edge_battery_t b;
    edge_battery_get(&b);
    cJSON *bj = cJSON_AddObjectToObject(o, "battery");
    cJSON_AddBoolToObject(bj, "present", b.present);
    if (b.present) {
        cJSON_AddNumberToObject(bj, "mv", b.mv);
        cJSON_AddNumberToObject(bj, "percent_approx", b.percent);
    }

    cJSON_AddNumberToObject(o, "heap_free", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(o, "heap_min", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));

    edge_wifi_info_t wi;
    edge_wifi_get_info(&wi);
    cJSON *wj = cJSON_AddObjectToObject(o, "link");
    cJSON_AddStringToObject(wj, "mode", link_name(cfg.link_mode));
    cJSON_AddStringToObject(wj, "state", edge_wifi_state_name(wi.state));
    cJSON_AddStringToObject(wj, "ssid", wi.ssid);
    cJSON_AddStringToObject(wj, "ip", wi.ip);
    cJSON_AddNumberToObject(wj, "stations", wi.stations);
    cJSON_AddNumberToObject(wj, "channel", wi.channel);
    cJSON_AddBoolToObject(wj, "sniffing", wi.sniffing);
    cJSON_AddBoolToObject(wj, "hopping", wi.hopping);
    if (wi.state == EDGE_WIFI_STA_CONNECTED) cJSON_AddNumberToObject(wj, "rssi", wi.rssi);

    edge_ble_info_t bi;
    edge_ble_get_info(&bi);
    cJSON *blj = cJSON_AddObjectToObject(o, "ble");
    cJSON_AddNumberToObject(blj, "connections", bi.connections);
    cJSON_AddNumberToObject(blj, "secured", bi.secured);
    cJSON_AddNumberToObject(blj, "bonds", bi.bonds);
    cJSON_AddBoolToObject(blj, "pairing_open", bi.pairing_open);
    if (bi.pairing_open && bi.pairing_left_s) cJSON_AddNumberToObject(blj, "pairing_left_s", bi.pairing_left_s);
    cJSON_AddBoolToObject(blj, "scanning", bi.scanning);

    sense_stats_t ss;
    sense_get_stats(&ss);
    cJSON *sj = cJSON_AddObjectToObject(o, "awareness");
    cJSON_AddBoolToObject(sj, "running", ss.running);
    cJSON *open = cJSON_AddObjectToObject(sj, "open_sightings");
    cJSON_AddNumberToObject(open, "ble", ss.ble_open);
    cJSON_AddNumberToObject(open, "wifi_ap", ss.ap_open);
    cJSON_AddNumberToObject(open, "wifi_probe", ss.probe_open);
    cJSON_AddNumberToObject(sj, "ble_reports", ss.ble_reports);
    cJSON_AddNumberToObject(sj, "wifi_frames", ss.wifi_frames);
    cJSON_AddNumberToObject(sj, "records", ss.records);
    cJSON_AddNumberToObject(sj, "queue_drops", ss.queue_drops);

    edge_log_stats_t ls;
    edge_log_stats(&ls);
    cJSON *lj = cJSON_AddObjectToObject(o, "log");
    cJSON_AddNumberToObject(lj, "head", ls.head_seq);
    cJSON_AddNumberToObject(lj, "oldest", ls.oldest_seq);
    cJSON_AddNumberToObject(lj, "count", ls.count);
    cJSON_AddNumberToObject(lj, "capacity", ls.capacity);
    cJSON_AddBoolToObject(lj, "erasing", ls.erasing);
    cJSON_AddNumberToObject(lj, "write_failures", ls.failed);

    cJSON_AddItemToObject(o, "mesh", mesh_status_json());
    cJSON_AddNumberToObject(o, "events_dropped", edge_events_dropped());
    return o;
}

static cJSON *config_json(const edge_config_t *c)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "mode", mode_name(c->mode));
    cJSON_AddStringToObject(o, "link", link_name(c->link_mode));
    cJSON *a = cJSON_AddObjectToObject(o, "awareness");
    cJSON_AddBoolToObject(a, "ble", c->ble_scan);
    cJSON_AddBoolToObject(a, "ble_active", c->ble_active);
    cJSON_AddBoolToObject(a, "wifi_aps", c->wifi_scan);
    cJSON_AddBoolToObject(a, "wifi_probes", c->wifi_probes);
    cJSON_AddBoolToObject(a, "probe_names", c->probe_ssid_plain);
    cJSON_AddNumberToObject(a, "rssi_floor", c->rssi_floor);
    cJSON_AddNumberToObject(a, "close_after_s", c->close_after_s);
    cJSON_AddNumberToObject(a, "max_window_s", c->max_window_s);
    cJSON *d = cJSON_AddObjectToObject(o, "display");
    cJSON_AddNumberToObject(d, "timeout_s", c->display_timeout_s);
    cJSON *m = cJSON_AddObjectToObject(o, "mesh");
    const mesh_region_t *r = mesh_region_by_code(c->region);
    cJSON_AddStringToObject(m, "region", r ? r->name : "UNSET");
    const mesh_preset_t *p = mesh_preset_by_code(c->preset);
    cJSON_AddStringToObject(m, "preset", p ? p->name : "LONG_FAST");
    cJSON_AddNumberToObject(m, "hop_limit", c->hop_limit);
    cJSON_AddStringToObject(m, "rebroadcast", rebroadcast_name(c->rebroadcast));
    cJSON_AddNumberToObject(m, "tx_power", c->tx_power);
    cJSON_AddNumberToObject(m, "channel_num", c->channel_num);
    char id[12];
    snprintf(id, sizeof(id), "!%08lx", (unsigned long)c->node_num);
    cJSON_AddStringToObject(m, "node_id", id);
    cJSON_AddStringToObject(m, "long_name", c->long_name);
    cJSON_AddStringToObject(m, "short_name", c->short_name);
    cJSON_AddNumberToObject(m, "nodeinfo_interval_s", c->nodeinfo_interval_s);
    cJSON *chs = cJSON_AddArrayToObject(m, "channels");
    for (int i = 0; i < c->n_channels; i++) {
        cJSON *ch = cJSON_CreateObject();
        cJSON_AddStringToObject(ch, "name", c->channels[i].name);
        char psk[64];
        psk_text(&c->channels[i], psk, sizeof(psk));
        cJSON_AddStringToObject(ch, "psk", psk);
        cJSON_AddItemToArray(chs, ch);
    }
    return o;
}

/* Applies a partial config object. Returns NULL on success or an error string. */
static const char *apply_config_json(const cJSON *j, edge_config_t *c)
{
    const char *s;
    double n;
    bool b;
    if ((s = jstr(j, "mode"))) {
        if (strcmp(s, "awareness") == 0) c->mode = EDGE_MODE_AWARENESS;
        else if (strcmp(s, "mesh") == 0) c->mode = EDGE_MODE_MESH;
        else return "mode must be awareness or mesh";
    }
    if ((s = jstr(j, "link"))) {
        if (strcmp(s, "auto") == 0) c->link_mode = EDGE_LINK_AUTO;
        else if (strcmp(s, "ap") == 0) c->link_mode = EDGE_LINK_AP;
        else if (strcmp(s, "sta") == 0) c->link_mode = EDGE_LINK_STA;
        else if (strcmp(s, "off") == 0) c->link_mode = EDGE_LINK_OFF;
        else return "link must be auto, ap, sta or off";
    }
    const cJSON *a = cJSON_GetObjectItemCaseSensitive(j, "awareness");
    if (cJSON_IsObject(a)) {
        if (jbool(a, "ble", &b)) c->ble_scan = b;
        if (jbool(a, "ble_active", &b)) c->ble_active = b;
        if (jbool(a, "wifi_aps", &b)) c->wifi_scan = b;
        if (jbool(a, "wifi_probes", &b)) c->wifi_probes = b;
        if (jbool(a, "probe_names", &b)) c->probe_ssid_plain = b;
        if (jnum(a, "rssi_floor", &n)) c->rssi_floor = (int8_t)(n < -127 ? -127 : n > 0 ? 0 : n);
        if (jnum(a, "close_after_s", &n)) c->close_after_s = (uint16_t)(n < 0 ? 0 : n > 3600 ? 3600 : n);
        if (jnum(a, "max_window_s", &n)) c->max_window_s = (uint16_t)(n < 0 ? 0 : n > 3600 ? 3600 : n);
    }
    const cJSON *d = cJSON_GetObjectItemCaseSensitive(j, "display");
    if (cJSON_IsObject(d) && jnum(d, "timeout_s", &n)) c->display_timeout_s = (uint16_t)(n < 0 ? 0 : n > 3600 ? 3600 : n);

    const cJSON *m = cJSON_GetObjectItemCaseSensitive(j, "mesh");
    if (cJSON_IsObject(m)) {
        if ((s = jstr(m, "region"))) {
            if (strcmp(s, "UNSET") == 0) {
                c->region = 0;
            } else {
                const mesh_region_t *r = mesh_region_by_name(s);
                if (!r) return "unknown region";
                c->region = r->code;
            }
        }
        if ((s = jstr(m, "preset"))) {
            const mesh_preset_t *p = mesh_preset_by_name(s);
            if (!p) return "unknown preset";
            c->preset = p->code;
        }
        if (jnum(m, "hop_limit", &n)) c->hop_limit = (uint8_t)(n < 1 ? 1 : n > 7 ? 7 : n);
        if ((s = jstr(m, "rebroadcast"))) {
            if (strcmp(s, "all") == 0) c->rebroadcast = EDGE_REBROADCAST_ALL;
            else if (strcmp(s, "local_only") == 0) c->rebroadcast = EDGE_REBROADCAST_LOCAL_ONLY;
            else if (strcmp(s, "none") == 0) c->rebroadcast = EDGE_REBROADCAST_NONE;
            else return "rebroadcast must be all, local_only or none";
        }
        if (jnum(m, "tx_power", &n)) c->tx_power = (int8_t)(n < 0 ? 0 : n > 30 ? 30 : n);
        if (jnum(m, "channel_num", &n)) c->channel_num = (uint16_t)(n < 0 ? 0 : n > 1000 ? 1000 : n);
        if ((s = jstr(m, "long_name"))) {
            if (!s[0] || strlen(s) >= sizeof(c->long_name)) return "long_name must be 1-39 bytes";
            edge_strlcpy(c->long_name, s, sizeof(c->long_name));
        }
        if ((s = jstr(m, "short_name"))) {
            if (!s[0] || strlen(s) >= sizeof(c->short_name)) return "short_name must be 1-4 bytes";
            edge_strlcpy(c->short_name, s, sizeof(c->short_name));
        }
        if (jnum(m, "nodeinfo_interval_s", &n)) c->nodeinfo_interval_s = (uint32_t)(n < 0 ? 0 : n);
        const cJSON *chs = cJSON_GetObjectItemCaseSensitive(m, "channels");
        if (cJSON_IsArray(chs)) {
            int cnt = cJSON_GetArraySize(chs);
            if (cnt < 1 || cnt > EDGE_MESH_MAX_CHANNELS) return "1 to 4 channels";
            edge_mesh_channel_t tmp[EDGE_MESH_MAX_CHANNELS];
            memset(tmp, 0, sizeof(tmp));
            for (int i = 0; i < cnt; i++) {
                const cJSON *ch = cJSON_GetArrayItem(chs, i);
                const char *name = jstr(ch, "name");
                if (name && strlen(name) >= sizeof(tmp[i].name)) return "channel name max 11 bytes";
                edge_strlcpy(tmp[i].name, name ? name : "", sizeof(tmp[i].name));
                const char *psk = jstr(ch, "psk");
                if (!parse_psk(psk ? psk : "default", &tmp[i])) return "psk must be default, none, simpleN or base64:<16|32 bytes>";
            }
            memcpy(c->channels, tmp, sizeof(tmp));
            c->n_channels = (uint8_t)cnt;
        }
    }
    return NULL;
}

/* ---- Sync over BLE ---------------------------------------------------------------- */

typedef struct {
    int conn;
    uint32_t from;
    uint32_t limit;
    cJSON *id;
} sync_job_t;

static void sync_task(void *arg)
{
    sync_job_t *j = arg;
    uint32_t next = j->from;
    int sent = edge_ble_stream_records(j->conn, j->from, j->limit, &next);
    edge_log_stats_t ls;
    edge_log_stats(&ls);
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "sync_done");
    if (j->id) cJSON_AddItemToObject(ev, "id", j->id);
    cJSON_AddBoolToObject(ev, "ok", sent >= 0);
    cJSON_AddNumberToObject(ev, "sent", sent < 0 ? 0 : sent);
    cJSON_AddNumberToObject(ev, "next", next);
    cJSON_AddNumberToObject(ev, "head", ls.head_seq);
    cJSON_AddBoolToObject(ev, "more", next <= ls.head_seq);
    edge_events_send_to(EDGE_SINK_BLE, j->conn, ev);
    cJSON_Delete(ev);
    free(j);
    s_sync_busy = false;
    vTaskDelete(NULL);
}

/* ---- Dispatcher --------------------------------------------------------------------- */

cJSON *edge_api_handle(const cJSON *req, edge_sink_t origin, int target)
{
    const char *cmd = jstr(req, "cmd");
    if (!cmd) return err_resp("missing cmd");
    cJSON *r = NULL;
    double n;
    bool b;

    if (strcmp(cmd, "info") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "info", edge_api_info_json());
    } else if (strcmp(cmd, "diag") == 0) {
        r = ok_resp();
        cJSON *h = cJSON_AddObjectToObject(r, "heap");
        cJSON_AddNumberToObject(h, "free", heap_caps_get_free_size(MALLOC_CAP_8BIT));
        cJSON_AddNumberToObject(h, "min", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
        cJSON_AddNumberToObject(h, "largest_block", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        UBaseType_t nt = uxTaskGetNumberOfTasks();
        TaskStatus_t *ts = malloc(nt * sizeof(TaskStatus_t));
        if (ts) {
            nt = uxTaskGetSystemState(ts, nt, NULL);
            cJSON *tasks = cJSON_AddObjectToObject(r, "stack_free_min");
            for (UBaseType_t i = 0; i < nt; i++) {
                cJSON_AddNumberToObject(tasks, ts[i].pcTaskName, (double)ts[i].usStackHighWaterMark);
            }
            free(ts);
        }
    } else if (strcmp(cmd, "status") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "status", edge_api_status_json());
    } else if (strcmp(cmd, "get_config") == 0) {
        edge_config_t c;
        edge_config_get(&c);
        r = ok_resp();
        cJSON_AddItemToObject(r, "config", config_json(&c));
    } else if (strcmp(cmd, "set_config") == 0 || strcmp(cmd, "set_mode") == 0) {
        const cJSON *cfgj = strcmp(cmd, "set_mode") == 0 ? req : cJSON_GetObjectItemCaseSensitive(req, "config");
        if (!cJSON_IsObject(cfgj)) return err_resp("missing config object");
        edge_config_t old, c;
        edge_config_get(&old);
        c = old;
        const char *e = apply_config_json(cfgj, &c);
        if (e) return err_resp(e);
        if (edge_config_set(&c) != ESP_OK) return err_resp("could not save settings");
        edge_config_get(&c);
        bool mesh_changed = old.region != c.region || old.preset != c.preset || old.hop_limit != c.hop_limit ||
                            old.tx_power != c.tx_power || old.channel_num != c.channel_num ||
                            old.n_channels != c.n_channels || old.rebroadcast != c.rebroadcast ||
                            memcmp(old.channels, c.channels, sizeof(c.channels)) != 0 ||
                            strcmp(old.long_name, c.long_name) != 0 || strcmp(old.short_name, c.short_name) != 0;
        bool sense_changed = old.ble_scan != c.ble_scan || old.ble_active != c.ble_active ||
                             old.wifi_scan != c.wifi_scan || old.wifi_probes != c.wifi_probes ||
                             old.rssi_floor != c.rssi_floor || old.close_after_s != c.close_after_s ||
                             old.max_window_s != c.max_window_s || old.probe_ssid_plain != c.probe_ssid_plain;
        if (old.link_mode != c.link_mode) edge_wifi_reconfigure();
        if (old.mode != c.mode) {
            edge_app_apply_mode((edge_mode_t)c.mode);
        } else {
            if (sense_changed) sense_reload();
            if (mesh_changed) mesh_reload();
        }
        edge_record_event(EDGE_EVT_CONFIG, old.mode != c.mode ? mode_name(c.mode) : "settings");
        r = ok_resp();
        cJSON_AddItemToObject(r, "config", config_json(&c));
    } else if (strcmp(cmd, "set_time") == 0) {
        if (!jnum(req, "unix_ms", &n)) return err_resp("missing unix_ms");
        bool was = edge_time_valid();
        const char *src = jstr(req, "source");
        edge_time_set_unix_ms((int64_t)n, src ? src : (origin == EDGE_SINK_BLE ? "phone" : "api"));
        if (!edge_time_valid()) return err_resp("time not plausible");
        if (!was) edge_record_event(EDGE_EVT_TIME_SET, edge_time_source());
        r = ok_resp();
    } else if (strcmp(cmd, "wifi_list") == 0 || strcmp(cmd, "wifi_scan") == 0) {
        if (strcmp(cmd, "wifi_scan") == 0) edge_wifi_request_scan();
        edge_wifi_net_t nets[EDGE_MAX_WIFI_NETS];
        int cnt = edge_wifi_nets_get(nets, EDGE_MAX_WIFI_NETS);
        r = ok_resp();
        cJSON *known = cJSON_AddArrayToObject(r, "known");
        for (int i = 0; i < cnt; i++) {
            cJSON *k = cJSON_CreateObject();
            cJSON_AddStringToObject(k, "ssid", nets[i].ssid);
            cJSON_AddBoolToObject(k, "has_password", nets[i].password[0] != 0);
            cJSON_AddItemToArray(known, k);
        }
        cJSON_AddItemToObject(r, "nearby", edge_wifi_scan_json());
    } else if (strcmp(cmd, "wifi_add") == 0) {
        const char *ssid = jstr(req, "ssid");
        const char *pw = jstr(req, "password");
        esp_err_t e = edge_wifi_net_add(ssid, pw ? pw : "");
        if (e != ESP_OK) return err_resp("ssid 1-32 bytes; password empty or 8-64 bytes");
        edge_wifi_reconfigure();
        r = ok_resp();
    } else if (strcmp(cmd, "wifi_remove") == 0) {
        const char *ssid = jstr(req, "ssid");
        if (!ssid || edge_wifi_net_remove(ssid) != ESP_OK) return err_resp("not found");
        edge_wifi_reconfigure();
        r = ok_resp();
    } else if (strcmp(cmd, "access") == 0) {
        const edge_identity_t *id = edge_identity();
        r = ok_resp();
        cJSON_AddStringToObject(r, "pin", id->access_pin);
        cJSON_AddStringToObject(r, "ap_ssid", id->ap_ssid);
        cJSON_AddStringToObject(r, "ap_password", id->ap_password);
    } else if (strcmp(cmd, "new_pin") == 0) {
        edge_identity_new_pin();
        r = ok_resp();
        cJSON_AddStringToObject(r, "pin", edge_identity()->access_pin);
    } else if (strcmp(cmd, "sync") == 0) {
        if (origin != EDGE_SINK_BLE) return err_resp("sync streams over Bluetooth; use GET /api/records over HTTP");
        if (s_sync_busy) return err_resp("a sync is already running");
        sync_job_t *j = calloc(1, sizeof(*j));
        if (!j) return err_resp("no memory");
        j->conn = target;
        j->from = jnum(req, "from", &n) ? (uint32_t)n : 0;
        j->limit = jnum(req, "limit", &n) && n > 0 ? (uint32_t)n : 0xffffffffu;
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(req, "id");
        j->id = id ? cJSON_Duplicate(id, true) : NULL;
        s_sync_busy = true;
        if (xTaskCreate(sync_task, "sync", 4096, j, 4, NULL) != pdPASS) {
            s_sync_busy = false;
            cJSON_Delete(j->id);
            free(j);
            return err_resp("no memory");
        }
        edge_log_stats_t ls;
        edge_log_stats(&ls);
        r = ok_resp();
        cJSON_AddNumberToObject(r, "oldest", ls.oldest_seq);
        cJSON_AddNumberToObject(r, "head", ls.head_seq);
    } else if (strcmp(cmd, "records") == 0) {
        uint32_t from = jnum(req, "from", &n) ? (uint32_t)n : 0;
        int limit = jnum(req, "limit", &n) ? (int)n : 20;
        if (limit < 1) limit = 1;
        if (limit > 20) limit = 20;
        edge_rec_t *recs = malloc(limit * sizeof(edge_rec_t));
        if (!recs) return err_resp("no memory");
        int cnt = edge_log_read_from(from, recs, limit);
        r = ok_resp();
        cJSON *arr = cJSON_AddArrayToObject(r, "records");
        for (int i = 0; i < cnt; i++) cJSON_AddItemToArray(arr, edge_rec_to_json(&recs[i]));
        cJSON_AddNumberToObject(r, "next", cnt ? recs[cnt - 1].seq + 1 : from);
        free(recs);
    } else if (strcmp(cmd, "log_erase") == 0) {
        if (!jbool(req, "confirm", &b) || !b) return err_resp("set confirm:true to erase every record");
        if (edge_log_erase(false) != ESP_OK) return err_resp("erase already running");
        r = ok_resp();
    } else if (strcmp(cmd, "watch") == 0) {
        const char *kind = jstr(req, "kind");
        uint8_t mac[6];
        if (!edge_mac_parse(jstr(req, "addr"), mac)) return err_resp("addr must be a MAC address");
        sense_watch_kind_t k = kind && strcmp(kind, "wifi") == 0 ? SENSE_WATCH_WIFI : SENSE_WATCH_BLE;
        sense_watch(k, mac, jnum(req, "seconds", &n) ? (uint16_t)n : 60);
        r = ok_resp();
    } else if (strcmp(cmd, "unwatch") == 0) {
        sense_unwatch();
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_send") == 0) {
        uint32_t to;
        if (!parse_node(cJSON_GetObjectItemCaseSensitive(req, "to"), &to)) return err_resp("bad destination");
        uint8_t ch = jnum(req, "channel", &n) ? (uint8_t)n : 0;
        bool ack = jbool(req, "want_ack", &b) ? b : true;
        uint32_t id = 0;
        esp_err_t e = mesh_send_text(to, ch, jstr(req, "text"), ack, &id);
        if (e == ESP_ERR_INVALID_STATE) return err_resp("mesh is not running or the LoRa region is not set");
        if (e == ESP_ERR_INVALID_SIZE) return err_resp("text must be 1-200 bytes");
        if (e != ESP_OK) return err_resp(esp_err_to_name(e));
        r = ok_resp();
        cJSON_AddNumberToObject(r, "packet_id", id);
    } else if (strcmp(cmd, "mesh_nodes") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "nodes", mesh_nodes_json());
    } else if (strcmp(cmd, "mesh_messages") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "messages", mesh_messages_json());
    } else if (strcmp(cmd, "mesh_status") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "mesh", mesh_status_json());
    } else if (strcmp(cmd, "mesh_new_identity") == 0) {
        edge_config_t c;
        edge_config_get(&c);
        c.node_num = 0; /* clamp() draws a new random number */
        edge_config_set(&c);
        mesh_reload();
        edge_config_get(&c);
        r = ok_resp();
        char id[12];
        snprintf(id, sizeof(id), "!%08lx", (unsigned long)c.node_num);
        cJSON_AddStringToObject(r, "node_id", id);
    } else if (strcmp(cmd, "regions") == 0) {
        r = ok_resp();
        cJSON *ra = cJSON_AddArrayToObject(r, "regions");
        for (size_t i = 0; mesh_region_at(i); i++) cJSON_AddItemToArray(ra, cJSON_CreateString(mesh_region_at(i)->name));
        cJSON *pa = cJSON_AddArrayToObject(r, "presets");
        for (size_t i = 0; mesh_preset_at(i); i++) cJSON_AddItemToArray(pa, cJSON_CreateString(mesh_preset_at(i)->name));
    } else if (strcmp(cmd, "pairing_open") == 0) {
        edge_ble_open_pairing(jnum(req, "seconds", &n) ? (uint32_t)n : 120);
        r = ok_resp();
    } else if (strcmp(cmd, "ble_forget_all") == 0) {
        if (!jbool(req, "confirm", &b) || !b) return err_resp("set confirm:true to remove every paired phone");
        edge_ble_forget_all();
        r = ok_resp();
    } else if (strcmp(cmd, "reboot") == 0) {
        edge_app_reboot_later(500);
        r = ok_resp();
    } else if (strcmp(cmd, "factory_reset") == 0) {
        if (!jbool(req, "confirm", &b) || !b) return err_resp("set confirm:true to erase settings, pairings and records");
        edge_app_factory_reset_later(500);
        r = ok_resp();
    } else {
        return err_resp("unknown cmd");
    }
    return r;
}

/* ---- Worker -------------------------------------------------------------------------- */

static void api_task(void *arg)
{
    job_t j;
    for (;;) {
        if (xQueueReceive(s_jobs, &j, portMAX_DELAY) != pdTRUE) continue;
        cJSON *req = cJSON_Parse(j.line);
        cJSON *resp;
        if (!cJSON_IsObject(req)) {
            resp = err_resp("invalid JSON");
        } else {
            resp = edge_api_handle(req, (edge_sink_t)j.sink, j.target);
            const cJSON *id = cJSON_GetObjectItemCaseSensitive(req, "id");
            if (id) cJSON_AddItemToObject(resp, "id", cJSON_Duplicate(id, true));
        }
        edge_events_send_to((edge_sink_t)j.sink, j.target, resp);
        cJSON_Delete(resp);
        cJSON_Delete(req);
        free(j.line);
    }
}

void edge_api_init(void)
{
    s_jobs = xQueueCreate(8, sizeof(job_t));
    xTaskCreate(api_task, "api", 5120, NULL, 4, NULL);
}

void edge_api_submit(edge_sink_t origin, int target, char *line)
{
    job_t j = {.sink = (uint8_t)origin, .target = target, .line = line};
    if (!s_jobs || xQueueSend(s_jobs, &j, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full");
        free(line);
    }
}
