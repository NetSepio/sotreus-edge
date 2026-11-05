#include "edge_records.h"

#include <stdio.h>
#include <string.h>

#include "edge_events.h"
#include "edge_time.h"
#include "edge_util.h"

static const char *type_name(uint8_t t)
{
    switch (t) {
    case EDGE_REC_BLE: return "ble";
    case EDGE_REC_WIFI_AP: return "wifi_ap";
    case EDGE_REC_WIFI_PROBE: return "wifi_probe";
    case EDGE_REC_MESH_TEXT: return "mesh_text";
    case EDGE_REC_EVENT: return "event";
    default: return "unknown";
    }
}

const char *edge_auth_name(uint8_t auth)
{
    switch (auth) {
    case EDGE_AUTH_OPEN: return "open";
    case EDGE_AUTH_WEP: return "wep";
    case EDGE_AUTH_WPA: return "wpa";
    case EDGE_AUTH_WPA2: return "wpa2";
    case EDGE_AUTH_WPA3: return "wpa3";
    case EDGE_AUTH_WPA2_WPA3: return "wpa2_wpa3";
    case EDGE_AUTH_ENTERPRISE: return "enterprise";
    case EDGE_AUTH_OWE: return "owe";
    default: return "unknown";
    }
}

static const char *event_name(uint16_t code)
{
    switch (code) {
    case EDGE_EVT_BOOT: return "boot";
    case EDGE_EVT_MODE: return "mode";
    case EDGE_EVT_TIME_SET: return "time_set";
    case EDGE_EVT_LOG_ERASED: return "log_erased";
    case EDGE_EVT_CONFIG: return "config";
    case EDGE_EVT_FIRMWARE: return "firmware";
    default: return "other";
    }
}

static void add_hex(cJSON *o, const char *key, const uint8_t *data, size_t n)
{
    char buf[2 * 224 + 1];
    if (n > 224) n = 224;
    edge_hex(data, n, buf);
    cJSON_AddStringToObject(o, key, buf);
}

static void add_mac(cJSON *o, const char *key, const uint8_t mac[6])
{
    char s[18];
    edge_mac_str(mac, s);
    cJSON_AddStringToObject(o, key, s);
}

static void add_rssi(cJSON *o, int8_t last, int8_t max, int8_t min)
{
    cJSON *r = cJSON_AddObjectToObject(o, "rssi");
    cJSON_AddNumberToObject(r, "last", last);
    cJSON_AddNumberToObject(r, "max", max);
    cJSON_AddNumberToObject(r, "min", min);
}

static void add_node(cJSON *o, const char *key, uint32_t n)
{
    char s[12];
    if (n == 0xffffffffu) {
        cJSON_AddStringToObject(o, key, "broadcast");
    } else {
        snprintf(s, sizeof(s), "!%08lx", (unsigned long)n);
        cJSON_AddStringToObject(o, key, s);
    }
}

/* Finds the Complete (0x09) or Shortened (0x08) Local Name AD structure. */
static bool ad_name(const uint8_t *ad, size_t n, char *out, size_t out_size)
{
    size_t i = 0;
    while (i + 1 < n) {
        uint8_t len = ad[i];
        if (len == 0 || i + 1 + len > n) break;
        uint8_t type = ad[i + 1];
        if (type == 0x09 || type == 0x08) {
            edge_sanitize_text(out, out_size, &ad[i + 2], len - 1);
            return true;
        }
        i += 1 + len;
    }
    return false;
}

static void ble_json(cJSON *o, const edge_body_ble_t *b)
{
    add_mac(o, "addr", b->addr);
    cJSON_AddStringToObject(o, "addr_type", b->addr_type == 0 ? "public" : "random");
    cJSON_AddNumberToObject(o, "pdu", b->evt_type);
    cJSON_AddBoolToObject(o, "connectable", (b->flags & EDGE_BLEF_CONNECTABLE) != 0);
    add_rssi(o, b->rssi_last, b->rssi_max, b->rssi_min);
    cJSON_AddNumberToObject(o, "count", b->count);
    uint8_t al = b->adv_len > 31 ? 31 : b->adv_len;
    uint8_t rl = b->rsp_len > 31 ? 31 : b->rsp_len;
    add_hex(o, "adv", b->adv, al);
    if (b->flags & EDGE_BLEF_SCAN_RSP) add_hex(o, "rsp", b->rsp, rl);
    char name[40];
    if (ad_name(b->adv, al, name, sizeof(name)) || ad_name(b->rsp, rl, name, sizeof(name))) {
        cJSON_AddStringToObject(o, "name", name);
    }
}

static void ap_json(cJSON *o, const edge_body_wifi_ap_t *b)
{
    add_mac(o, "bssid", b->bssid);
    if (b->flags & EDGE_APF_HIDDEN || b->ssid_len == 0) {
        cJSON_AddNullToObject(o, "ssid");
    } else {
        char ssid[33 * 2];
        edge_sanitize_text(ssid, sizeof(ssid), b->ssid, b->ssid_len > 32 ? 32 : b->ssid_len);
        cJSON_AddStringToObject(o, "ssid", ssid);
    }
    cJSON_AddBoolToObject(o, "hidden", (b->flags & EDGE_APF_HIDDEN) != 0);
    cJSON_AddNumberToObject(o, "channel", b->channel);
    cJSON_AddStringToObject(o, "auth", edge_auth_name(b->auth));
    add_rssi(o, b->rssi_last, b->rssi_max, b->rssi_min);
    cJSON_AddNumberToObject(o, "count", b->count);
    if (b->beacon_itvl) cJSON_AddNumberToObject(o, "beacon_tu", b->beacon_itvl);
    cJSON_AddBoolToObject(o, "scan_only", (b->flags & EDGE_APF_FROM_SCAN) != 0);
    if (b->flags & EDGE_APF_REMOTE_ID) {
        add_hex(o, "remote_id", b->rid, b->rid_len > sizeof(b->rid) ? sizeof(b->rid) : b->rid_len);
    }
}

static void probe_json(cJSON *o, const edge_body_wifi_probe_t *b)
{
    add_mac(o, "mac", b->mac);
    cJSON_AddBoolToObject(o, "randomized", (b->flags & EDGE_PRBF_RANDOMIZED) != 0);
    cJSON_AddBoolToObject(o, "wildcard", (b->flags & EDGE_PRBF_WILDCARD) != 0);
    cJSON_AddNumberToObject(o, "channel", b->channel);
    add_rssi(o, b->rssi_last, b->rssi_max, b->rssi_min);
    cJSON_AddNumberToObject(o, "count", b->count);
    int n = b->n_ssids > EDGE_PROBE_MAX_SSIDS ? EDGE_PROBE_MAX_SSIDS : b->n_ssids;
    cJSON *hashes = cJSON_AddArrayToObject(o, "ssid_hashes");
    for (int i = 0; i < n; i++) {
        char h[9];
        snprintf(h, sizeof(h), "%08lx", (unsigned long)b->ssid_hash[i]);
        cJSON_AddItemToArray(hashes, cJSON_CreateString(h));
    }
    if (b->flags & EDGE_PRBF_PLAIN) {
        cJSON *names = cJSON_AddArrayToObject(o, "ssids");
        for (int i = 0; i < n; i++) {
            char s[33 * 2];
            uint8_t l = b->plain[i].len > 32 ? 32 : b->plain[i].len;
            edge_sanitize_text(s, sizeof(s), b->plain[i].ssid, l);
            cJSON_AddItemToArray(names, cJSON_CreateString(s));
        }
    }
    cJSON_AddBoolToObject(o, "more_ssids", (b->flags & EDGE_PRBF_MORE) != 0);
}

static void mesh_json(cJSON *o, const edge_body_mesh_text_t *b)
{
    add_node(o, "from", b->from);
    add_node(o, "to", b->to);
    cJSON_AddNumberToObject(o, "packet_id", b->packet_id);
    cJSON_AddNumberToObject(o, "channel", b->channel);
    cJSON_AddStringToObject(o, "dir", b->dir ? "sent" : "received");
    if (!b->dir) {
        cJSON_AddNumberToObject(o, "rssi", b->rssi);
        cJSON_AddNumberToObject(o, "snr", b->snr_x4 / 4.0);
        cJSON_AddNumberToObject(o, "hop_limit", b->hops & 0x0f);
        cJSON_AddNumberToObject(o, "hop_start", b->hops >> 4);
    }
    char text[EDGE_MESH_TEXT_MAX + 1];
    uint8_t l = b->text_len > EDGE_MESH_TEXT_MAX ? EDGE_MESH_TEXT_MAX : b->text_len;
    edge_sanitize_text(text, sizeof(text), (const uint8_t *)b->text, l);
    cJSON_AddStringToObject(o, "text", text);
}

static void event_json(cJSON *o, const edge_body_event_t *b)
{
    cJSON_AddStringToObject(o, "event", event_name(b->code));
    char d[sizeof(b->detail) + 1];
    uint8_t l = b->detail_len > sizeof(b->detail) ? sizeof(b->detail) : b->detail_len;
    edge_sanitize_text(d, sizeof(d), (const uint8_t *)b->detail, l);
    cJSON_AddStringToObject(o, "detail", d);
}

cJSON *edge_rec_to_json(const edge_rec_t *rec)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "seq", rec->seq);
    cJSON_AddStringToObject(o, "type", type_name(rec->type));
    cJSON_AddStringToObject(o, "provenance", "SENSED");
    bool tv = (rec->flags & EDGE_RECF_TIME_VALID) != 0;
    cJSON_AddBoolToObject(o, "time_valid", tv);
    /* Time fields keep their meaning explicit: unix ms, or uptime ms of `boot`. */
    cJSON_AddNumberToObject(o, tv ? "first_unix_ms" : "first_uptime_ms", (double)rec->first_ms);
    cJSON_AddNumberToObject(o, tv ? "last_unix_ms" : "last_uptime_ms", (double)rec->last_ms);
    cJSON_AddNumberToObject(o, "boot", rec->boot);
    if (rec->flags & EDGE_RECF_CONTINUES) cJSON_AddTrueToObject(o, "continues");
    if (rec->flags & EDGE_RECF_TRUNCATED) cJSON_AddTrueToObject(o, "truncated");

    switch (rec->type) {
    case EDGE_REC_BLE:
        cJSON_AddStringToObject(o, "origin", "EdgeBle");
        ble_json(o, (const edge_body_ble_t *)rec->body);
        break;
    case EDGE_REC_WIFI_AP:
        cJSON_AddStringToObject(o, "origin", "EdgeWifi");
        ap_json(o, (const edge_body_wifi_ap_t *)rec->body);
        break;
    case EDGE_REC_WIFI_PROBE:
        cJSON_AddStringToObject(o, "origin", "EdgeWifi");
        probe_json(o, (const edge_body_wifi_probe_t *)rec->body);
        break;
    case EDGE_REC_MESH_TEXT:
        cJSON_ReplaceItemInObject(o, "provenance", cJSON_CreateString("NETWORK"));
        cJSON_AddStringToObject(o, "origin", "EdgeLora");
        mesh_json(o, (const edge_body_mesh_text_t *)rec->body);
        break;
    case EDGE_REC_EVENT:
        cJSON_AddStringToObject(o, "origin", "Edge");
        event_json(o, (const edge_body_event_t *)rec->body);
        break;
    default:
        break;
    }
    return o;
}

void edge_record_event(edge_event_code_t code, const char *detail)
{
    edge_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.type = EDGE_REC_EVENT;
    rec.first_ms = rec.last_ms = edge_uptime_ms();
    edge_body_event_t *b = (edge_body_event_t *)rec.body;
    b->code = code;
    size_t l = detail ? strlen(detail) : 0;
    if (l > sizeof(b->detail)) {
        l = sizeof(b->detail);
        rec.flags |= EDGE_RECF_TRUNCATED;
    }
    if (l) memcpy(b->detail, detail, l);
    b->detail_len = (uint8_t)l;
    rec.body_len = sizeof(*b);
    if (edge_log_append(&rec) == ESP_OK) {
        edge_events_publish_record(&rec);
    }
}
