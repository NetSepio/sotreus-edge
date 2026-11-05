#include "edge_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

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
#include "edge_meshmod.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "edge_wifi.h"
#include "mesh.h"
#include "mesh_pb.h"
#include "mesh_radio.h"
#include <math.h>
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

static const char *const REBROADCAST[] = {"all", "all_skip_decoding", "local_only", "known_only", "none", "core_portnums_only"};
static const char *const ROLES[] = {"CLIENT", "CLIENT_MUTE", "ROUTER", NULL, "REPEATER", "TRACKER", "SENSOR",
                                    "TAK", "CLIENT_HIDDEN", "LOST_AND_FOUND", "TAK_TRACKER", "ROUTER_LATE"};
static const char *const POS_MODES[] = {"off", "fixed", "client"};
static const char *const NOTIFY_MODES[] = {"off", "message", "bell"};

static const char *rebroadcast_name(uint8_t r)
{
    return r < 6 ? REBROADCAST[r] : "all";
}

static int name_index(const char *s, const char *const *names, int n)
{
    for (int i = 0; s && i < n; i++) {
        if (names[i] && strcasecmp(s, names[i]) == 0) return i;
    }
    return -1;
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
    cJSON_AddStringToObject(o, "vendor", "NetSepio LLC");
    cJSON_AddStringToObject(o, "vendor_url", "https://netsepio.com");
    cJSON_AddStringToObject(o, "product_url", "https://sotreus.com");
    char host[24];
    snprintf(host, sizeof(host), "sotreus-%s.local", id->suffix);
    for (char *c = host; *c; c++) *c = (char)((*c >= 'A' && *c <= 'Z') ? *c + 32 : *c);
    cJSON_AddStringToObject(o, "mdns_host", host);
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
    edge_config_t *cfg = malloc(sizeof(*cfg));
    if (!cfg) return cJSON_CreateObject();
    edge_config_get(cfg);
    cJSON *o = cJSON_CreateObject();
    int64_t up = edge_uptime_ms();
    cJSON_AddNumberToObject(o, "uptime_ms", (double)up);
    cJSON_AddBoolToObject(o, "time_valid", edge_time_valid());
    cJSON_AddStringToObject(o, "time_source", edge_time_source());
    if (edge_time_valid()) cJSON_AddNumberToObject(o, "unix_ms", (double)edge_time_to_unix_ms(up));
    cJSON_AddStringToObject(o, "mode", mode_name(cfg->mode));

    edge_battery_t b;
    edge_battery_get(&b);
    cJSON *bj = cJSON_AddObjectToObject(o, "battery");
    cJSON_AddBoolToObject(bj, "present", b.present);
    cJSON_AddStringToObject(bj, "source", b.present ? "battery" : b.external ? "usb" : "unknown");
    if (b.present) {
        cJSON_AddNumberToObject(bj, "mv", b.mv);
        cJSON_AddNumberToObject(bj, "percent_approx", b.percent);
    }

    cJSON_AddNumberToObject(o, "heap_free", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(o, "heap_min", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));

    edge_wifi_info_t wi;
    edge_wifi_get_info(&wi);
    cJSON *wj = cJSON_AddObjectToObject(o, "link");
    cJSON_AddStringToObject(wj, "mode", link_name(cfg->link_mode));
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
    free(cfg);
    return o;
}

static const char *const TAK_TEAMS[] = {"default", "White", "Yellow", "Orange", "Magenta", "Red", "Maroon", "Purple",
                                        "Dark Blue", "Blue", "Cyan", "Teal", "Green", "Dark Green", "Brown"};
static const char *const TAK_ROLES[] = {"default", "Team Member", "Team Lead", "HQ", "Sniper", "Medic", "Forward Observer",
                                        "RTO", "K9"};
static const char *const AUDIO_RATES[] = {"default", "3200", "2400", "1600", "1400", "1300", "1200", "700", "700B", "700C", "450"};

static void mod_json(cJSON *m, const edge_mesh_mod_t *d, int n_channels)
{
    cJSON *chs = cJSON_GetObjectItemCaseSensitive(m, "channels");
    for (int i = 0; chs && i < n_channels; i++) {
        cJSON *ch = cJSON_GetArrayItem(chs, i);
        if (!ch) break;
        cJSON_AddBoolToObject(ch, "uplink", (d->ch_uplink >> i) & 1);
        cJSON_AddBoolToObject(ch, "downlink", (d->ch_downlink >> i) & 1);
    }
    cJSON *a = cJSON_AddObjectToObject(m, "admin");
    cJSON_AddBoolToObject(a, "enabled", d->admin_enabled);
    cJSON_AddStringToObject(a, "mode", d->admin_mode == EDGE_ADMIN_KEYS ? "keys" : "open");
    cJSON_AddBoolToObject(a, "admin_channel", d->admin_channel);
    cJSON *keys = cJSON_AddArrayToObject(a, "keys");
    for (int i = 0; i < d->n_admin_keys; i++) {
        char b64[48];
        edge_b64_encode(d->admin_keys[i], 32, b64, sizeof(b64));
        cJSON_AddItemToArray(keys, cJSON_CreateString(b64));
    }
    cJSON *rh = cJSON_AddObjectToObject(m, "remote_hardware");
    cJSON_AddBoolToObject(rh, "enabled", d->rhw_enabled);
    cJSON_AddBoolToObject(rh, "allow_undefined_pins", d->rhw_undefined_ok);
    cJSON *pins = cJSON_AddArrayToObject(rh, "pins");
    for (int i = 0; i < d->n_pins; i++) {
        cJSON *p = cJSON_CreateObject();
        cJSON_AddNumberToObject(p, "gpio", d->pins[i].gpio);
        cJSON_AddStringToObject(p, "name", d->pins[i].name);
        cJSON_AddStringToObject(p, "type", d->pins[i].type == EDGE_RHW_WRITE ? "write" : "read");
        cJSON_AddItemToArray(pins, p);
    }
    cJSON *sf = cJSON_AddObjectToObject(m, "store_forward");
    cJSON_AddBoolToObject(sf, "server", d->sf_enabled);
    cJSON_AddBoolToObject(sf, "heartbeat", d->sf_heartbeat);
    cJSON_AddNumberToObject(sf, "return_max", d->sf_return_max);
    cJSON_AddNumberToObject(sf, "return_window_min", d->sf_return_window_min);
    cJSON *q = cJSON_AddObjectToObject(m, "mqtt");
    cJSON_AddBoolToObject(q, "enabled", d->mqtt_enabled);
    cJSON_AddStringToObject(q, "address", d->mqtt_address);
    cJSON_AddStringToObject(q, "username", d->mqtt_username);
    cJSON_AddBoolToObject(q, "password_set", d->mqtt_password[0] != 0);
    cJSON_AddBoolToObject(q, "encryption", d->mqtt_encryption);
    cJSON_AddBoolToObject(q, "json", d->mqtt_json);
    cJSON_AddBoolToObject(q, "tls", d->mqtt_tls);
    cJSON_AddStringToObject(q, "root", d->mqtt_root);
    cJSON_AddBoolToObject(q, "proxy_to_client", d->mqtt_proxy_to_client);
    cJSON *mp = cJSON_AddObjectToObject(q, "map");
    cJSON_AddBoolToObject(mp, "enabled", d->mqtt_map);
    cJSON_AddNumberToObject(mp, "interval_s", d->mqtt_map_interval_s);
    cJSON_AddNumberToObject(mp, "precision", d->mqtt_map_precision);
    cJSON_AddBoolToObject(mp, "location", d->mqtt_map_location);
    cJSON *tk = cJSON_AddObjectToObject(m, "tak");
    cJSON_AddStringToObject(tk, "team", TAK_TEAMS[d->tak_team < 15 ? d->tak_team : 0]);
    cJSON_AddStringToObject(tk, "role", TAK_ROLES[d->tak_role < 9 ? d->tak_role : 0]);
    cJSON *au = cJSON_AddObjectToObject(m, "audio");
    cJSON_AddBoolToObject(au, "enabled", d->audio_enabled);
    cJSON_AddNumberToObject(au, "ptt_gpio", d->audio_ptt);
    cJSON_AddStringToObject(au, "bitrate", AUDIO_RATES[d->audio_bitrate < 11 ? d->audio_bitrate : 0]);
    cJSON_AddNumberToObject(au, "i2s_ws", d->i2s_ws);
    cJSON_AddNumberToObject(au, "i2s_sd", d->i2s_sd);
    cJSON_AddNumberToObject(au, "i2s_din", d->i2s_din);
    cJSON_AddNumberToObject(au, "i2s_sck", d->i2s_sck);
}

static int8_t gpio_arg(double n)
{
    return (int8_t)(n < 0 || n > 48 ? -1 : n);
}

static const char *apply_mod_json(const cJSON *m, edge_mesh_mod_t *d)
{
    const char *s;
    double n;
    bool b;
    const cJSON *chs = cJSON_GetObjectItemCaseSensitive(m, "channels");
    if (cJSON_IsArray(chs)) {
        uint8_t up = 0, down = 0;
        int cnt = cJSON_GetArraySize(chs);
        for (int i = 0; i < cnt && i < EDGE_MESH_MAX_CHANNELS; i++) {
            const cJSON *ch = cJSON_GetArrayItem(chs, i);
            if (jbool(ch, "uplink", &b) && b) up |= (uint8_t)(1u << i);
            if (jbool(ch, "downlink", &b) && b) down |= (uint8_t)(1u << i);
        }
        d->ch_uplink = up;
        d->ch_downlink = down;
    }
    const cJSON *a = cJSON_GetObjectItemCaseSensitive(m, "admin");
    if (cJSON_IsObject(a)) {
        if (jbool(a, "enabled", &b)) d->admin_enabled = b;
        if ((s = jstr(a, "mode"))) {
            if (strcmp(s, "open") == 0) d->admin_mode = EDGE_ADMIN_OPEN;
            else if (strcmp(s, "keys") == 0) d->admin_mode = EDGE_ADMIN_KEYS;
            else return "admin mode must be open or keys";
        }
        if (jbool(a, "admin_channel", &b)) d->admin_channel = b;
        const cJSON *keys = cJSON_GetObjectItemCaseSensitive(a, "keys");
        if (cJSON_IsArray(keys)) {
            int cnt = cJSON_GetArraySize(keys);
            if (cnt > EDGE_ADMIN_KEYS_MAX) return "at most 3 admin keys";
            uint8_t nk = 0;
            for (int i = 0; i < cnt; i++) {
                const cJSON *k = cJSON_GetArrayItem(keys, i);
                if (!cJSON_IsString(k) || !k->valuestring[0]) continue;
                uint8_t raw[40];
                const char *v = k->valuestring;
                if (strncmp(v, "base64:", 7) == 0) v += 7;
                if (edge_b64_decode(v, raw, sizeof(raw)) != 32) return "an admin key is a node's 32-byte public key in base64";
                memcpy(d->admin_keys[nk++], raw, 32);
            }
            d->n_admin_keys = nk;
        }
        if (d->admin_mode == EDGE_ADMIN_KEYS && !d->n_admin_keys && !d->admin_channel)
            return "add an admin key (or allow the admin channel) before limiting remote admin to keys";
    }
    const cJSON *rh = cJSON_GetObjectItemCaseSensitive(m, "remote_hardware");
    if (cJSON_IsObject(rh)) {
        if (jbool(rh, "enabled", &b)) d->rhw_enabled = b;
        if (jbool(rh, "allow_undefined_pins", &b)) d->rhw_undefined_ok = b;
        const cJSON *pins = cJSON_GetObjectItemCaseSensitive(rh, "pins");
        if (cJSON_IsArray(pins)) {
            int cnt = cJSON_GetArraySize(pins);
            if (cnt > EDGE_RHW_PINS_MAX) return "at most 8 remote hardware pins";
            d->n_pins = 0;
            for (int i = 0; i < cnt; i++) {
                const cJSON *p = cJSON_GetArrayItem(pins, i);
                if (!jnum(p, "gpio", &n)) continue;
                if (edge_pin_protected((int)n)) return "that GPIO is used by the board itself";
                edge_rhw_pin_t *pin = &d->pins[d->n_pins++];
                memset(pin, 0, sizeof(*pin));
                pin->gpio = (uint8_t)n;
                pin->type = (s = jstr(p, "type")) && strcmp(s, "write") == 0 ? EDGE_RHW_WRITE : EDGE_RHW_READ;
                if ((s = jstr(p, "name"))) edge_strlcpy(pin->name, s, sizeof(pin->name));
            }
        }
    }
    const cJSON *sf = cJSON_GetObjectItemCaseSensitive(m, "store_forward");
    if (cJSON_IsObject(sf)) {
        if (jbool(sf, "server", &b)) d->sf_enabled = b;
        if (jbool(sf, "heartbeat", &b)) d->sf_heartbeat = b;
        if (jnum(sf, "return_max", &n)) d->sf_return_max = (uint16_t)(n < 1 ? 1 : n > 100 ? 100 : n);
        if (jnum(sf, "return_window_min", &n)) d->sf_return_window_min = (uint16_t)(n < 1 ? 1 : n > 10080 ? 10080 : n);
    }
    const cJSON *q = cJSON_GetObjectItemCaseSensitive(m, "mqtt");
    if (cJSON_IsObject(q)) {
        if (jbool(q, "enabled", &b)) d->mqtt_enabled = b;
        if ((s = jstr(q, "address"))) {
            if (strlen(s) >= sizeof(d->mqtt_address) || strpbrk(s, " /")) return "MQTT server: host or host:port, up to 63 characters";
            edge_strlcpy(d->mqtt_address, s, sizeof(d->mqtt_address));
        }
        if ((s = jstr(q, "username"))) edge_strlcpy(d->mqtt_username, s, sizeof(d->mqtt_username));
        if ((s = jstr(q, "password"))) edge_strlcpy(d->mqtt_password, s, sizeof(d->mqtt_password));
        if (jbool(q, "encryption", &b)) d->mqtt_encryption = b;
        if (jbool(q, "json", &b)) d->mqtt_json = b;
        if (jbool(q, "tls", &b)) d->mqtt_tls = b;
        if ((s = jstr(q, "root"))) {
            if (strlen(s) >= sizeof(d->mqtt_root) || strpbrk(s, "+#")) return "MQTT root topic: up to 31 characters, no + or #";
            edge_strlcpy(d->mqtt_root, s, sizeof(d->mqtt_root));
        }
        if (jbool(q, "proxy_to_client", &b)) d->mqtt_proxy_to_client = b;
        const cJSON *mp = cJSON_GetObjectItemCaseSensitive(q, "map");
        if (cJSON_IsObject(mp)) {
            if (jbool(mp, "enabled", &b)) d->mqtt_map = b;
            if (jnum(mp, "interval_s", &n)) d->mqtt_map_interval_s = (uint32_t)(n < 900 ? 900 : n);
            if (jnum(mp, "precision", &n)) d->mqtt_map_precision = (uint8_t)(n < 10 ? 10 : n > 16 ? 16 : n);
            if (jbool(mp, "location", &b)) d->mqtt_map_location = b;
        }
    }
    const cJSON *tk = cJSON_GetObjectItemCaseSensitive(m, "tak");
    if (cJSON_IsObject(tk)) {
        if ((s = jstr(tk, "team"))) {
            int i = name_index(s, TAK_TEAMS, 15);
            if (i < 0) return "unknown TAK team";
            d->tak_team = (uint8_t)i;
        }
        if ((s = jstr(tk, "role"))) {
            int i = name_index(s, TAK_ROLES, 9);
            if (i < 0) return "unknown TAK role";
            d->tak_role = (uint8_t)i;
        }
    }
    const cJSON *au = cJSON_GetObjectItemCaseSensitive(m, "audio");
    if (cJSON_IsObject(au)) {
        if (jbool(au, "enabled", &b)) d->audio_enabled = b;
        if (jnum(au, "ptt_gpio", &n)) d->audio_ptt = gpio_arg(n);
        if ((s = jstr(au, "bitrate"))) {
            int i = name_index(s, AUDIO_RATES, 11);
            if (i < 0) return "unknown audio bitrate";
            d->audio_bitrate = (uint8_t)i;
        }
        if (jnum(au, "i2s_ws", &n)) d->i2s_ws = gpio_arg(n);
        if (jnum(au, "i2s_sd", &n)) d->i2s_sd = gpio_arg(n);
        if (jnum(au, "i2s_din", &n)) d->i2s_din = gpio_arg(n);
        if (jnum(au, "i2s_sck", &n)) d->i2s_sck = gpio_arg(n);
        int8_t g[] = {d->audio_ptt, d->i2s_ws, d->i2s_sd, d->i2s_din, d->i2s_sck};
        for (int i = 0; i < 5; i++) {
            if (g[i] >= 0 && edge_pin_protected(g[i])) return "an audio GPIO is used by the board itself";
        }
    }
    return NULL;
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
        cJSON_AddNumberToObject(ch, "position_precision", c->channels[i].position_precision);
        cJSON_AddItemToArray(chs, ch);
    }
    const edge_mesh_ext_t *x = &c->mx;
    cJSON_AddStringToObject(m, "role", ROLES[x->role] ? ROLES[x->role] : "CLIENT");
    cJSON_AddBoolToObject(m, "licensed", x->licensed);
    cJSON *pos = cJSON_AddObjectToObject(m, "position");
    cJSON_AddStringToObject(pos, "mode", POS_MODES[x->pos_mode]);
    if (x->pos_lat_i || x->pos_lon_i) {
        cJSON_AddNumberToObject(pos, "lat", x->pos_lat_i / 1e7);
        cJSON_AddNumberToObject(pos, "lon", x->pos_lon_i / 1e7);
        cJSON_AddNumberToObject(pos, "alt", x->pos_alt);
    }
    cJSON_AddNumberToObject(pos, "interval_s", x->pos_interval_s);
    cJSON_AddNumberToObject(m, "telemetry_interval_s", x->telemetry_interval_s);
    cJSON *nb = cJSON_AddObjectToObject(m, "neighbor_info");
    cJSON_AddBoolToObject(nb, "enabled", x->neighbor_info);
    cJSON_AddNumberToObject(nb, "interval_s", x->neighbor_interval_s);
    cJSON_AddNumberToObject(m, "range_test_interval_s", x->range_test_interval_s);
    cJSON *nt = cJSON_AddObjectToObject(m, "notify");
    cJSON_AddStringToObject(nt, "mode", NOTIFY_MODES[x->notify_mode]);
    cJSON_AddNumberToObject(nt, "gpio", x->notify_gpio);
    cJSON *dt = cJSON_AddObjectToObject(m, "detection");
    cJSON_AddBoolToObject(dt, "enabled", x->detect_enabled);
    cJSON_AddNumberToObject(dt, "gpio", x->detect_gpio);
    cJSON_AddBoolToObject(dt, "active_high", x->detect_active_high);
    cJSON_AddNumberToObject(dt, "min_interval_s", x->detect_min_interval_s);
    cJSON_AddStringToObject(dt, "name", x->detect_name);
    cJSON *px = cJSON_AddObjectToObject(m, "paxcounter");
    cJSON_AddBoolToObject(px, "enabled", x->pax_enabled);
    cJSON_AddNumberToObject(px, "interval_s", x->pax_interval_s);
    cJSON *se = cJSON_AddObjectToObject(m, "serial");
    cJSON_AddBoolToObject(se, "enabled", x->serial_enabled);
    cJSON_AddNumberToObject(se, "rx", x->serial_rx);
    cJSON_AddNumberToObject(se, "tx", x->serial_tx);
    cJSON_AddNumberToObject(se, "baud", x->serial_baud);
    cJSON *cn = cJSON_AddArrayToObject(m, "canned");
    for (int i = 0; i < x->n_canned; i++) cJSON_AddItemToArray(cn, cJSON_CreateString(x->canned[i]));
    cJSON *lo = cJSON_AddObjectToObject(m, "lora");
    cJSON_AddNumberToObject(lo, "freq_offset_hz", x->freq_offset_hz);
    cJSON_AddNumberToObject(lo, "override_freq_mhz", x->override_freq_khz / 1000.0);
    cJSON_AddBoolToObject(lo, "tx_enabled", x->tx_enabled);
    cJSON_AddBoolToObject(lo, "override_duty_cycle", x->override_duty_cycle);
    cJSON_AddBoolToObject(lo, "rx_boosted", x->rx_boosted);
    cJSON_AddBoolToObject(lo, "ignore_mqtt", x->ignore_mqtt);
    cJSON_AddBoolToObject(lo, "ok_to_mqtt", x->ok_to_mqtt);
    cJSON *ig = cJSON_AddArrayToObject(m, "ignore");
    for (int i = 0; i < x->n_ignore; i++) {
        char nid[12];
        snprintf(nid, sizeof(nid), "!%08lx", (unsigned long)x->ignore[i]);
        cJSON_AddItemToArray(ig, cJSON_CreateString(nid));
    }
    edge_mesh_mod_t *md = malloc(sizeof(*md));
    if (md) {
        edge_mod_get(md);
        mod_json(m, md, c->n_channels);
        free(md);
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
            int i = name_index(s, REBROADCAST, 6);
            if (i < 0) return "rebroadcast must be all, all_skip_decoding, local_only, known_only, none or core_portnums_only";
            c->rebroadcast = (uint8_t)i;
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
            if (cnt < 1 || cnt > EDGE_MESH_MAX_CHANNELS) return "1 to 8 channels";
            edge_mesh_channel_t tmp[EDGE_MESH_MAX_CHANNELS];
            memset(tmp, 0, sizeof(tmp));
            for (int i = 0; i < cnt; i++) {
                const cJSON *ch = cJSON_GetArrayItem(chs, i);
                const char *name = jstr(ch, "name");
                if (name && strlen(name) >= sizeof(tmp[i].name)) return "channel name max 11 bytes";
                edge_strlcpy(tmp[i].name, name ? name : "", sizeof(tmp[i].name));
                const char *psk = jstr(ch, "psk");
                if (!parse_psk(psk ? psk : "default", &tmp[i])) return "psk must be default, none, simpleN or base64:<16|32 bytes>";
                double pp;
                bool pub = tmp[i].psk_len == 1 && tmp[i].psk[0] == 1;
                tmp[i].position_precision = jnum(ch, "position_precision", &pp) ? (uint8_t)(pp < 0 ? 0 : pp > 32 ? 32 : pp)
                                                                                : (pub ? 13 : 32);
            }
            memcpy(c->channels, tmp, sizeof(tmp));
            c->n_channels = (uint8_t)cnt;
        }
        edge_mesh_ext_t *x = &c->mx;
        if ((s = jstr(m, "role"))) {
            int i = name_index(s, ROLES, 12);
            if (i < 0) return "unknown role";
            x->role = (uint8_t)i;
        }
        if (jbool(m, "licensed", &b)) x->licensed = b;
        const cJSON *pos = cJSON_GetObjectItemCaseSensitive(m, "position");
        if (cJSON_IsObject(pos)) {
            if ((s = jstr(pos, "mode"))) {
                int i = name_index(s, POS_MODES, 3);
                if (i < 0) return "position mode must be off, fixed or client";
                x->pos_mode = (uint8_t)i;
            }
            double la, lo2;
            if (jnum(pos, "lat", &la) && jnum(pos, "lon", &lo2)) {
                if (la < -90 || la > 90 || lo2 < -180 || lo2 > 180) return "position out of range";
                x->pos_lat_i = (int32_t)lround(la * 1e7);
                x->pos_lon_i = (int32_t)lround(lo2 * 1e7);
            }
            if (jnum(pos, "alt", &n)) x->pos_alt = (int32_t)n;
            if (jnum(pos, "interval_s", &n)) x->pos_interval_s = (uint32_t)(n < 0 ? 0 : n);
        }
        if (jnum(m, "telemetry_interval_s", &n)) x->telemetry_interval_s = (uint32_t)(n < 0 ? 0 : n);
        const cJSON *nb = cJSON_GetObjectItemCaseSensitive(m, "neighbor_info");
        if (cJSON_IsObject(nb)) {
            if (jbool(nb, "enabled", &b)) x->neighbor_info = b;
            if (jnum(nb, "interval_s", &n)) x->neighbor_interval_s = (uint32_t)(n < 0 ? 0 : n);
        }
        if (jnum(m, "range_test_interval_s", &n)) x->range_test_interval_s = (uint32_t)(n < 0 ? 0 : n);
        const cJSON *nt = cJSON_GetObjectItemCaseSensitive(m, "notify");
        if (cJSON_IsObject(nt)) {
            if ((s = jstr(nt, "mode"))) {
                int i = name_index(s, NOTIFY_MODES, 3);
                if (i < 0) return "notify mode must be off, message or bell";
                x->notify_mode = (uint8_t)i;
            }
            if (jnum(nt, "gpio", &n)) x->notify_gpio = (int8_t)(n < -1 ? -1 : n > 48 ? -1 : n);
        }
        const cJSON *dt = cJSON_GetObjectItemCaseSensitive(m, "detection");
        if (cJSON_IsObject(dt)) {
            if (jbool(dt, "enabled", &b)) x->detect_enabled = b;
            if (jnum(dt, "gpio", &n)) x->detect_gpio = (int8_t)(n < -1 ? -1 : n > 48 ? -1 : n);
            if (jbool(dt, "active_high", &b)) x->detect_active_high = b;
            if (jnum(dt, "min_interval_s", &n)) x->detect_min_interval_s = (uint16_t)(n < 0 ? 0 : n > 3600 ? 3600 : n);
            if ((s = jstr(dt, "name"))) edge_strlcpy(x->detect_name, s, sizeof(x->detect_name));
        }
        const cJSON *px = cJSON_GetObjectItemCaseSensitive(m, "paxcounter");
        if (cJSON_IsObject(px)) {
            if (jbool(px, "enabled", &b)) x->pax_enabled = b;
            if (jnum(px, "interval_s", &n)) x->pax_interval_s = (uint32_t)(n < 0 ? 0 : n);
        }
        const cJSON *se = cJSON_GetObjectItemCaseSensitive(m, "serial");
        if (cJSON_IsObject(se)) {
            if (jbool(se, "enabled", &b)) x->serial_enabled = b;
            if (jnum(se, "rx", &n)) x->serial_rx = (int8_t)(n < -1 ? -1 : n > 48 ? -1 : n);
            if (jnum(se, "tx", &n)) x->serial_tx = (int8_t)(n < -1 ? -1 : n > 48 ? -1 : n);
            if (jnum(se, "baud", &n)) x->serial_baud = (uint32_t)n;
        }
        const cJSON *cn = cJSON_GetObjectItemCaseSensitive(m, "canned");
        if (cJSON_IsArray(cn)) {
            int cnt = cJSON_GetArraySize(cn);
            if (cnt > EDGE_CANNED_MAX) return "at most 8 canned messages";
            x->n_canned = 0;
            for (int i = 0; i < cnt; i++) {
                const cJSON *it = cJSON_GetArrayItem(cn, i);
                if (!cJSON_IsString(it) || !it->valuestring[0]) continue;
                if (strlen(it->valuestring) >= sizeof(x->canned[0])) return "canned messages are at most 39 bytes";
                edge_strlcpy(x->canned[x->n_canned++], it->valuestring, sizeof(x->canned[0]));
            }
        }
        const cJSON *lo = cJSON_GetObjectItemCaseSensitive(m, "lora");
        if (cJSON_IsObject(lo)) {
            if (jnum(lo, "freq_offset_hz", &n)) x->freq_offset_hz = (int32_t)n;
            if (jnum(lo, "override_freq_mhz", &n)) x->override_freq_khz = (uint32_t)(n <= 0 ? 0 : lround(n * 1000));
            if (jbool(lo, "tx_enabled", &b)) x->tx_enabled = b;
            if (jbool(lo, "override_duty_cycle", &b)) x->override_duty_cycle = b;
            if (jbool(lo, "rx_boosted", &b)) x->rx_boosted = b;
            if (jbool(lo, "ignore_mqtt", &b)) x->ignore_mqtt = b;
            if (jbool(lo, "ok_to_mqtt", &b)) x->ok_to_mqtt = b;
        }
        const cJSON *ig = cJSON_GetObjectItemCaseSensitive(m, "ignore");
        if (cJSON_IsArray(ig)) {
            int cnt = cJSON_GetArraySize(ig);
            if (cnt > EDGE_IGNORE_MAX) return "at most 8 ignored nodes";
            x->n_ignore = 0;
            for (int i = 0; i < cnt; i++) {
                uint32_t num;
                if (parse_node(cJSON_GetArrayItem(ig, i), &num) && num != MT_BROADCAST) x->ignore[x->n_ignore++] = num;
            }
        }
    }
    return NULL;
}

/* ---- Meshtastic channel link (https://meshtastic.org/e/#<base64url ChannelSet>) ---------- */

static void channel_url(const edge_config_t *c, char *out, size_t cap)
{
    uint8_t up = 0, down = 0;
    {
        edge_mesh_mod_t *md = malloc(sizeof(*md));
        if (md) {
            edge_mod_get(md);
            up = md->ch_uplink;
            down = md->ch_downlink;
            free(md);
        }
    }
    mt_channelset_t cs = {0};
    cs.n = c->n_channels;
    for (int i = 0; i < c->n_channels; i++) {
        edge_strlcpy(cs.ch[i].name, c->channels[i].name, sizeof(cs.ch[i].name));
        memcpy(cs.ch[i].psk, c->channels[i].psk, c->channels[i].psk_len);
        cs.ch[i].psk_len = c->channels[i].psk_len;
        cs.ch[i].position_precision = c->channels[i].position_precision;
        cs.ch[i].uplink = (up >> i) & 1;
        cs.ch[i].downlink = (down >> i) & 1;
    }
    cs.has_lora = true;
    cs.use_preset = true;
    cs.preset = c->preset;
    cs.region = c->region;
    cs.hop_limit = c->hop_limit;
    cs.tx_enabled = c->mx.tx_enabled;
    cs.tx_power = c->tx_power;
    cs.channel_num = c->channel_num;
    uint8_t *buf = malloc(512 + 700);
    if (!buf) { out[0] = 0; return; }
    char *b64 = (char *)buf + 512;
    size_t n = mt_channelset_encode(&cs, buf, 512);
    int l = edge_b64_encode(buf, n, b64, 700);
    for (int i = 0; i < l; i++) {
        if (b64[i] == '+') b64[i] = '-';
        if (b64[i] == '/') b64[i] = '_';
        if (b64[i] == '=') b64[i] = '\0';
    }
    snprintf(out, cap, "https://meshtastic.org/e/#%s", b64);
    free(buf);
}

/* Applies a channel link: replace = all channels and LoRa settings, otherwise add its channels. */
static const char *apply_channel_url(const char *url, bool replace, edge_config_t *c)
{
    const char *h = url ? strchr(url, '#') : NULL;
    h = h ? h + 1 : url;
    if (!h || !*h) return "not a channel link";
    uint8_t buf[256]; /* a full 8-channel set with 32-byte keys is about 420 bytes of base64 */
    int n = edge_b64_decode(h, buf, sizeof(buf));
    mt_channelset_t cs;
    if (n <= 0 || !mt_channelset_decode(buf, (size_t)n, &cs)) return "the link could not be read";
    int base = replace ? 0 : c->n_channels;
    if (base + cs.n > EDGE_MESH_MAX_CHANNELS) return "that would make more than 8 channels";
    for (int i = 0; i < cs.n; i++) {
        edge_mesh_channel_t *ch = &c->channels[base + i];
        memset(ch, 0, sizeof(*ch));
        edge_strlcpy(ch->name, cs.ch[i].name, sizeof(ch->name));
        uint8_t len = cs.ch[i].psk_len;
        if (len != 0 && len != 1 && len != 16 && len != 32) return "unsupported key length in the link";
        memcpy(ch->psk, cs.ch[i].psk, len);
        ch->psk_len = len;
        bool pub = len == 1 && cs.ch[i].psk[0] == 1;
        ch->position_precision = (uint8_t)(cs.ch[i].position_precision ? cs.ch[i].position_precision : (pub ? 13 : 32));
    }
    edge_mesh_mod_t *md = malloc(sizeof(*md));
    if (md) {
        edge_mod_get(md);
        if (replace) md->ch_uplink = md->ch_downlink = 0;
        for (int i = 0; i < cs.n; i++) {
            uint8_t bit = (uint8_t)(1u << (base + i));
            md->ch_uplink = (uint8_t)(cs.ch[i].uplink ? md->ch_uplink | bit : md->ch_uplink & ~bit);
            md->ch_downlink = (uint8_t)(cs.ch[i].downlink ? md->ch_downlink | bit : md->ch_downlink & ~bit);
        }
        edge_mod_set(md);
        free(md);
    }
    c->n_channels = (uint8_t)(base + cs.n);
    if (replace && cs.has_lora) {
        if (cs.use_preset && mesh_preset_by_code((uint8_t)cs.preset)) c->preset = (uint8_t)cs.preset;
        if (cs.region && mesh_region_by_code((uint8_t)cs.region)) c->region = (uint8_t)cs.region;
        if (cs.hop_limit) c->hop_limit = (uint8_t)cs.hop_limit;
        c->channel_num = (uint16_t)cs.channel_num;
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

static __attribute__((noinline)) cJSON *cmd_set_config(const cJSON *cfgj)
{
    edge_config_t *old = malloc(2 * sizeof(*old));
    if (!old) return err_resp("out of memory");
    edge_config_t *c = old + 1;
    edge_config_get(old);
    *c = *old;
    const char *e = apply_config_json(cfgj, c);
    if (e) {
        free(old);
        return err_resp(e);
    }
    bool mod_changed = false;
    const cJSON *meshj = cJSON_GetObjectItemCaseSensitive(cfgj, "mesh");
    if (cJSON_IsObject(meshj)) {
        edge_mesh_mod_t *md = malloc(2 * sizeof(*md));
        if (!md) {
            free(old);
            return err_resp("out of memory");
        }
        edge_mod_get(md);
        md[1] = md[0];
        e = apply_mod_json(meshj, &md[1]);
        if (e) {
            free(md);
            free(old);
            return err_resp(e);
        }
        mod_changed = memcmp(&md[0], &md[1], sizeof(md[0])) != 0;
        if (mod_changed) edge_mod_set(&md[1]);
        free(md);
    }
    if (edge_config_set(c) != ESP_OK) {
        free(old);
        return err_resp("could not save settings");
    }
    edge_config_get(c);
    bool mesh_changed = old->region != c->region || old->preset != c->preset || old->hop_limit != c->hop_limit ||
                        old->tx_power != c->tx_power || old->channel_num != c->channel_num ||
                        old->n_channels != c->n_channels || old->rebroadcast != c->rebroadcast ||
                        memcmp(old->channels, c->channels, sizeof(c->channels)) != 0 ||
                        memcmp(&old->mx, &c->mx, sizeof(c->mx)) != 0 || old->nodeinfo_interval_s != c->nodeinfo_interval_s ||
                        strcmp(old->long_name, c->long_name) != 0 || strcmp(old->short_name, c->short_name) != 0 ||
                        mod_changed;
    bool sense_changed = old->ble_scan != c->ble_scan || old->ble_active != c->ble_active ||
                         old->wifi_scan != c->wifi_scan || old->wifi_probes != c->wifi_probes ||
                         old->rssi_floor != c->rssi_floor || old->close_after_s != c->close_after_s ||
                         old->max_window_s != c->max_window_s || old->probe_ssid_plain != c->probe_ssid_plain;
    if (old->link_mode != c->link_mode) edge_wifi_reconfigure();
    if (old->mode != c->mode) {
        edge_app_apply_mode((edge_mode_t)c->mode);
    } else {
        if (sense_changed) sense_reload();
        if (mesh_changed) mesh_reload();
    }
    edge_record_event(EDGE_EVT_CONFIG, old->mode != c->mode ? mode_name(c->mode) : "settings");
    cJSON *r = ok_resp();
    cJSON_AddItemToObject(r, "config", config_json(c));
    free(old);
    return r;
}

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
    } else if (strcmp(cmd, "ctx_get") == 0) {
        char *buf = malloc(EDGE_CTX_MAX);
        if (!buf) return err_resp("no memory");
        size_t n = edge_ctx_get(buf, EDGE_CTX_MAX);
        r = ok_resp();
        cJSON *c = n ? cJSON_Parse(buf) : NULL;
        cJSON_AddItemToObject(r, "ctx", cJSON_IsObject(c) ? c : cJSON_CreateObject());
        if (c && !cJSON_IsObject(c)) cJSON_Delete(c);
        free(buf);
    } else if (strcmp(cmd, "ctx_set") == 0) {
        const cJSON *c = cJSON_GetObjectItemCaseSensitive(req, "ctx");
        if (!cJSON_IsObject(c)) return err_resp("ctx must be an object");
        char *s = cJSON_PrintUnformatted(c);
        if (!s) return err_resp("no memory");
        esp_err_t e = edge_ctx_set(s);
        free(s);
        if (e == ESP_ERR_INVALID_SIZE) return err_resp("context settings are too large");
        if (e != ESP_OK) return err_resp("could not save");
        r = ok_resp();
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
        edge_config_t *c = malloc(sizeof(*c));
        if (!c) return err_resp("out of memory");
        edge_config_get(c);
        r = ok_resp();
        cJSON_AddItemToObject(r, "config", config_json(c));
        free(c);
    } else if (strcmp(cmd, "set_config") == 0 || strcmp(cmd, "set_mode") == 0) {
        const cJSON *cfgj = strcmp(cmd, "set_mode") == 0 ? req : cJSON_GetObjectItemCaseSensitive(req, "config");
        if (!cJSON_IsObject(cfgj)) return err_resp("missing config object");
        r = cmd_set_config(cfgj);
    } else if (strcmp(cmd, "set_time") == 0) {
        if (!jnum(req, "unix_ms", &n)) return err_resp("missing unix_ms");
        bool was = edge_time_valid();
        const char *src = jstr(req, "source");
        edge_time_set_unix_ms((int64_t)n, src ? src : (origin == EDGE_SINK_BLE ? "phone" : "api"));
        if (jnum(req, "tz_offset_min", &n)) edge_time_set_tz_offset_min((int)n);
        if (!edge_time_valid()) return err_resp("time not plausible");
        (void)was;
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
        cJSON_AddBoolToObject(r, "scanning", strcmp(cmd, "wifi_scan") == 0 || edge_wifi_scanning());
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
        uint32_t reply = jnum(req, "reply_id", &n) ? (uint32_t)n : 0;
        uint32_t emoji = jbool(req, "reaction", &b) && b ? 1 : 0;
        esp_err_t e = mesh_send_text(to, ch, jstr(req, "text"), ack, reply, emoji, &id);
        if (e == ESP_ERR_INVALID_STATE) return err_resp("mesh is not running or the LoRa region is not set");
        if (e == ESP_ERR_INVALID_SIZE) return err_resp("text must be 1-200 bytes");
        if (e != ESP_OK) return err_resp(esp_err_to_name(e));
        r = ok_resp();
        cJSON_AddNumberToObject(r, "packet_id", id);
    } else if (strcmp(cmd, "mesh_canned") == 0) {
        uint32_t id = 0;
        esp_err_t e = mesh_send_canned(jnum(req, "index", &n) ? (int)n : 0, &id);
        if (e != ESP_OK) return err_resp(e == ESP_ERR_INVALID_STATE ? "mesh is not running" : "no such canned message");
        r = ok_resp();
        cJSON_AddNumberToObject(r, "packet_id", id);
    } else if (strcmp(cmd, "mesh_traceroute") == 0) {
        uint32_t to, id = 0;
        if (!parse_node(cJSON_GetObjectItemCaseSensitive(req, "to"), &to) || to == MT_BROADCAST) return err_resp("pick a node");
        esp_err_t e = mesh_traceroute(to, jnum(req, "channel", &n) ? (uint8_t)n : 0, &id);
        if (e != ESP_OK) return err_resp(e == ESP_ERR_INVALID_STATE ? "mesh is not running" : esp_err_to_name(e));
        r = ok_resp();
        cJSON_AddNumberToObject(r, "packet_id", id);
    } else if (strcmp(cmd, "mesh_request_position") == 0) {
        uint32_t to;
        if (!parse_node(cJSON_GetObjectItemCaseSensitive(req, "to"), &to) || to == MT_BROADCAST) return err_resp("pick a node");
        if (mesh_request_position(to, jnum(req, "channel", &n) ? (uint8_t)n : 0) != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_waypoint") == 0) {
        mt_waypoint_t w = {0};
        double la, lo2;
        if (!jnum(req, "lat", &la) || !jnum(req, "lon", &lo2) || la < -90 || la > 90 || lo2 < -180 || lo2 > 180) return err_resp("lat and lon required");
        w.has_lat = w.has_lon = true;
        w.lat_i = (int32_t)lround(la * 1e7);
        w.lon_i = (int32_t)lround(lo2 * 1e7);
        edge_strlcpy(w.name, jstr(req, "name") ? jstr(req, "name") : "Waypoint", sizeof(w.name));
        if (jstr(req, "description")) edge_strlcpy(w.description, jstr(req, "description"), sizeof(w.description));
        if (jnum(req, "expire_unix_s", &n)) w.expire = (uint32_t)n;
        if (jnum(req, "icon", &n)) w.icon = (uint32_t)n;
        if (jnum(req, "id", &n)) w.id = (uint32_t)n;
        uint32_t id = 0;
        if (mesh_send_waypoint(&w, jnum(req, "channel", &n) ? (uint8_t)n : 0, &id) != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
        cJSON_AddNumberToObject(r, "waypoint_id", id);
    } else if (strcmp(cmd, "mesh_waypoints") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "waypoints", mesh_waypoints_json());
    } else if (strcmp(cmd, "mesh_store_forward") == 0) {
        uint32_t router = 0;
        const cJSON *rv = cJSON_GetObjectItemCaseSensitive(req, "router");
        if (rv && !parse_node(rv, &router)) return err_resp("bad router id");
        esp_err_t e = mesh_sf_request(router == MT_BROADCAST ? 0 : router, jnum(req, "channel", &n) ? (uint8_t)n : 0);
        if (e == ESP_ERR_NOT_FOUND) return err_resp("no store & forward router heard yet");
        if (e != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_send_nodeinfo") == 0) {
        if (mesh_send_nodeinfo_now() != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_set_position") == 0) {
        double la, lo2, al = 0;
        if (!jnum(req, "lat", &la) || !jnum(req, "lon", &lo2) || la < -90 || la > 90 || lo2 < -180 || lo2 > 180) return err_resp("lat and lon required");
        jnum(req, "alt", &al);
        mesh_set_client_position((int32_t)lround(la * 1e7), (int32_t)lround(lo2 * 1e7), (int32_t)al);
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_node") == 0) {
        uint32_t num;
        if (!parse_node(cJSON_GetObjectItemCaseSensitive(req, "id"), &num) || num == MT_BROADCAST) return err_resp("bad node id");
        int fav = jbool(req, "favorite", &b) ? (b ? 1 : 0) : -1;
        int ign = jbool(req, "ignored", &b) ? (b ? 1 : 0) : -1;
        if (ign >= 0) { /* the ignore list lives in settings so it survives restarts */
            edge_config_t *c = malloc(sizeof(*c));
            if (!c) return err_resp("out of memory");
            edge_config_get(c);
            int k = 0;
            for (int i = 0; i < c->mx.n_ignore; i++) {
                if (c->mx.ignore[i] != num) c->mx.ignore[k++] = c->mx.ignore[i];
            }
            c->mx.n_ignore = (uint8_t)k;
            if (ign && c->mx.n_ignore < EDGE_IGNORE_MAX) c->mx.ignore[c->mx.n_ignore++] = num;
            edge_config_set(c);
            free(c);
            mesh_reload();
        }
        mesh_node_flags(num, fav, ign);
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_channel_url") == 0) {
        edge_config_t *c = malloc(sizeof(*c));
        char *out = malloc(800);
        if (!c || !out) {
            free(c);
            free(out);
            return err_resp("out of memory");
        }
        edge_config_get(c);
        const char *url = jstr(req, "url");
        if (url) {
            bool replace = jbool(req, "replace", &b) ? b : true;
            const char *e = apply_channel_url(url, replace, c);
            if (e) {
                free(c);
                free(out);
                return err_resp(e);
            }
            edge_config_set(c);
            mesh_reload();
            edge_config_get(c);
        }
        channel_url(c, out, 800);
        r = ok_resp();
        cJSON_AddStringToObject(r, "url", out);
        free(c);
        free(out);
    } else if (strcmp(cmd, "mesh_tak") == 0) {
        r = ok_resp();
        cJSON_AddItemToObject(r, "tak", mesh_tak_json());
    } else if (strcmp(cmd, "mesh_tak_chat") == 0) {
        esp_err_t e = mesh_tak_send_chat(jstr(req, "text"), jnum(req, "channel", &n) ? (uint8_t)n : 0);
        if (e == ESP_ERR_INVALID_ARG) return err_resp("text: 1 to 200 bytes");
        if (e != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
    } else if (strcmp(cmd, "mesh_tak_position") == 0) {
        esp_err_t e = mesh_tak_send_position(jnum(req, "channel", &n) ? (uint8_t)n : 0);
        if (e == ESP_ERR_NOT_FOUND) return err_resp("no position to send: set a fixed position, or have the app share one, and allow position on that channel");
        if (e != ESP_OK) return err_resp("mesh is not running");
        r = ok_resp();
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
        edge_config_t *c = malloc(sizeof(*c));
        if (!c) return err_resp("out of memory");
        edge_config_get(c);
        c->node_num = 0; /* clamp() draws a new random number */
        edge_config_set(c);
        edge_pki_regenerate(); /* and a new key pair */
        mesh_reload();
        edge_config_get(c);
        r = ok_resp();
        char id[12];
        snprintf(id, sizeof(id), "!%08lx", (unsigned long)c->node_num);
        cJSON_AddStringToObject(r, "node_id", id);
        free(c);
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
    xTaskCreate(api_task, "api", 6144, NULL, 4, NULL);
}

void edge_api_submit(edge_sink_t origin, int target, char *line)
{
    job_t j = {.sink = (uint8_t)origin, .target = target, .line = line};
    if (!s_jobs || xQueueSend(s_jobs, &j, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full");
        free(line);
    }
}
