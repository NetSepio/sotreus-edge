#include "edge_wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mdns.h"
#include "sdkconfig.h"

#include "edge_config.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "mesh.h"
#include "sense.h"

static const char *TAG = "wifi";

#define TICK_MS             100
#define HOP_DWELL_MS        200
#define CONNECT_TIMEOUT_MS  20000
#define AP_RESCAN_MS        (5 * 60 * 1000)
#define STA_RETRY_MS        30000
#define SENSE_SCAN_MS       45000
#define MAX_FAILS           3
#define MAX_NEARBY          24
#define NEARBY_TTL_MS       (5 * 60 * 1000)
#define SWEEP_HOME_MS       600     /* time back on the link channel between swept channels */
#define SCAN_STUCK_MS       20000
#define SENSE_SCAN_STA_MS   120000

/* Networks heard recently, from scans and from beacons the sniffer hears. */
typedef struct {
    char ssid[33];
    int8_t rssi;
    uint8_t channel;
    uint8_t auth;
    int64_t seen;
} near_t;

typedef struct {
    char ssid[33];
    uint8_t channel;
    int8_t rssi;
} cand_t;

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static esp_netif_t *s_sta_netif, *s_ap_netif;

static volatile edge_wifi_state_t s_state = EDGE_WIFI_OFF;
static volatile bool s_reconfig = true;
static volatile bool s_scan_busy;
static volatile bool s_scan_done;
static volatile bool s_scan_for_link;
static volatile bool s_scan_requested;
static volatile bool s_got_ip;
static volatile bool s_disconnected;
static volatile int s_stations;
static volatile bool s_sniff_aps, s_sniff_probes;
static bool s_promisc_on;
static bool s_hopping;
static uint8_t s_cur_channel;
static int s_hop_step;
static int64_t s_last_hop, s_last_link_scan, s_last_sense_scan, s_connect_started, s_retry_at;
static int s_fails;
static int s_cand_idx;
static char s_ip[16];
static char s_joined[33];
static uint8_t s_ap_mac[6], s_sta_mac[6];
static near_t s_near[MAX_NEARBY];
static portMUX_TYPE s_near_mux = portMUX_INITIALIZER_UNLOCKED;
static cand_t s_cand[EDGE_MAX_WIFI_NETS];
static int s_ncand;
static bool s_scan_in_flight;
static bool s_link_scan_complete;
static uint8_t s_sweep_ch;      /* next channel of a channel-by-channel sweep, 0 = none */
static int64_t s_sweep_next_at, s_scan_issued, s_scan_started, s_last_scan_done;

static void near_update(const uint8_t *ssid, size_t len, int8_t rssi, uint8_t channel, uint8_t auth)
{
    if (len == 0 || len > 32 || ssid[0] == 0) return;
    int64_t t = edge_uptime_ms();
    portENTER_CRITICAL(&s_near_mux);
    int slot = -1, oldest = 0;
    for (int i = 0; i < MAX_NEARBY; i++) {
        if (s_near[i].ssid[0] && strlen(s_near[i].ssid) == len && memcmp(s_near[i].ssid, ssid, len) == 0) {
            slot = i;
            break;
        }
        if (s_near[i].seen < s_near[oldest].seen) oldest = i;
    }
    bool fresh = slot < 0;
    if (fresh) slot = oldest;
    near_t *e = &s_near[slot];
    /* Keep the strongest access point per name while it is recent. */
    if (fresh || rssi >= e->rssi || t - e->seen > 30000) {
        memcpy(e->ssid, ssid, len);
        e->ssid[len] = '\0';
        e->rssi = rssi;
        e->channel = channel;
        e->auth = auth;
    }
    e->seen = t;
    portEXIT_CRITICAL(&s_near_mux);
}

const char *edge_wifi_state_name(edge_wifi_state_t s)
{
    switch (s) {
    case EDGE_WIFI_OFF: return "off";
    case EDGE_WIFI_AP: return "ap";
    case EDGE_WIFI_STA_SEARCHING: return "searching";
    case EDGE_WIFI_STA_CONNECTING: return "connecting";
    case EDGE_WIFI_STA_CONNECTED: return "connected";
    case EDGE_WIFI_STA_WAITING: return "waiting";
    }
    return "unknown";
}

static int64_t now_ms(void)
{
    return edge_uptime_ms();
}

/* ---- 802.11 management frame parsing (runs in the Wi-Fi driver task) ------- */

static const uint8_t OUI_MS[3] = {0x00, 0x50, 0xf2};
static const uint8_t OUI_RID[3] = {0xfa, 0x0b, 0xbc}; /* ASD-STAN / ASTM F3411 Remote ID */

static uint8_t rsn_auth(const uint8_t *ie, uint8_t len)
{
    /* version(2) group(4) pairwise_count(2) pairwise(4n) akm_count(2) akm(4m) */
    if (len < 8) return EDGE_AUTH_WPA2;
    size_t p = 6;
    uint16_t pc = ie[p] | (ie[p + 1] << 8);
    p += 2 + 4u * pc;
    if (p + 2 > len) return EDGE_AUTH_WPA2;
    uint16_t ac = ie[p] | (ie[p + 1] << 8);
    p += 2;
    bool psk = false, sae = false, ent = false, owe = false;
    for (uint16_t i = 0; i < ac && p + 4 <= len; i++, p += 4) {
        if (ie[p] != 0x00 || ie[p + 1] != 0x0f || ie[p + 2] != 0xac) continue;
        switch (ie[p + 3]) {
        case 1: case 3: case 5: case 11: case 12: case 13: ent = true; break;
        case 2: case 4: case 6: psk = true; break;
        case 8: case 9: sae = true; break;
        case 18: owe = true; break;
        default: break;
        }
    }
    if (ent) return EDGE_AUTH_ENTERPRISE;
    if (psk && sae) return EDGE_AUTH_WPA2_WPA3;
    if (sae) return EDGE_AUTH_WPA3;
    if (owe) return EDGE_AUTH_OWE;
    return EDGE_AUTH_WPA2;
}

static void parse_mgmt(const uint8_t *f, size_t len, int8_t rssi, uint8_t rx_channel)
{
    if (len < 24) return;
    uint8_t fc = f[0];
    if (((fc >> 2) & 3) != 0) return; /* management only */
    uint8_t subtype = (fc >> 4) & 0x0f;
    const uint8_t *sa = f + 10;
    if (memcmp(sa, s_ap_mac, 6) == 0 || memcmp(sa, s_sta_mac, 6) == 0) return;

    if (subtype == 4) { /* probe request */
        if (!s_sniff_probes) return;
        if (sa[0] & 0x01) return; /* multicast source is invalid */
        sense_probe_report_t r = {.channel = rx_channel, .rssi = rssi};
        memcpy(r.mac, sa, 6);
        size_t p = 24;
        while (p + 2 <= len) {
            uint8_t id = f[p], l = f[p + 1];
            if (p + 2 + l > len) break;
            if (id == 0) {
                r.ssid_len = l > 32 ? 32 : l;
                memcpy(r.ssid, &f[p + 2], r.ssid_len);
                break;
            }
            p += 2 + l;
        }
        sense_feed_probe(&r);
        mesh_pax_seen(true, r.mac);
        return;
    }
    if (subtype != 8 && subtype != 5) return; /* beacon / probe response */
    if (!s_sniff_aps || len < 36) return;

    sense_ap_report_t r = {.channel = rx_channel, .rssi = rssi};
    memcpy(r.bssid, f + 16, 6);
    r.beacon_itvl = f[32] | (f[33] << 8);
    uint16_t cap = f[34] | (f[35] << 8);
    if (subtype == 5) r.flags |= EDGE_APF_PROBE_RESP;
    bool rsn = false, wpa = false;
    uint8_t auth = EDGE_AUTH_OPEN;
    size_t p = 36;
    bool ssid_seen = false;
    while (p + 2 <= len) {
        uint8_t id = f[p], l = f[p + 1];
        const uint8_t *ie = &f[p + 2];
        if (p + 2 + l > len) break;
        switch (id) {
        case 0: {
            ssid_seen = true;
            r.ssid_len = l > 32 ? 32 : l;
            memcpy(r.ssid, ie, r.ssid_len);
            /* Hidden networks send an empty or all-zero name. */
            bool blank = true;
            for (int i = 0; i < r.ssid_len; i++) blank &= ie[i] == 0;
            if (blank) {
                r.flags |= EDGE_APF_HIDDEN;
                r.ssid_len = 0;
            }
            break;
        }
        case 3:
            if (l >= 1 && ie[0] >= 1 && ie[0] <= 14) r.channel = ie[0];
            break;
        case 48:
            rsn = true;
            auth = rsn_auth(ie, l);
            break;
        case 221:
            if (l >= 4 && memcmp(ie, OUI_MS, 3) == 0 && ie[3] == 1) wpa = true;
            if (l >= 3 && memcmp(ie, OUI_RID, 3) == 0) {
                r.rid_len = l > sizeof(r.rid) ? sizeof(r.rid) : l;
                memcpy(r.rid, ie, r.rid_len);
            }
            break;
        default:
            break;
        }
        p += 2 + l;
    }
    if (!ssid_seen) r.flags |= EDGE_APF_HIDDEN;
    if (!rsn) auth = wpa ? EDGE_AUTH_WPA : (cap & 0x0010) ? EDGE_AUTH_WEP : EDGE_AUTH_OPEN;
    r.auth = auth;
    if (!(r.flags & EDGE_APF_HIDDEN)) near_update(r.ssid, r.ssid_len, rssi, r.channel, auth);
    sense_feed_ap(&r);
}

static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) return;
    const wifi_promiscuous_pkt_t *pkt = buf;
    size_t len = pkt->rx_ctrl.sig_len;
    if (len > 4) len -= 4; /* FCS */
    parse_mgmt(pkt->payload, len, (int8_t)pkt->rx_ctrl.rssi, (uint8_t)pkt->rx_ctrl.channel);
}

/* ---- Events ----------------------------------------------------------------- */

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_SCAN_DONE:
            s_scan_done = true;
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d = data;
            ESP_LOGI(TAG, "station disconnected (reason %d)", d ? d->reason : -1);
            s_disconnected = true;
            s_got_ip = false;
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED:
            s_stations++;
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            if (s_stations > 0) s_stations--;
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_got_ip = true;
    }
    if (s_task) xTaskNotifyGive(s_task);
}

/* ---- Link helpers ------------------------------------------------------------ */

/* A client on our access point, or our own station link, would lose its
 * connection during a full 13-channel passive scan (~5-10 s off channel). */
static bool link_in_use(void)
{
    return (s_state == EDGE_WIFI_AP && s_stations > 0) || s_state == EDGE_WIFI_STA_CONNECTED;
}

static bool issue_scan(uint8_t channel)
{
    wifi_scan_config_t sc = {
        .channel = channel, /* 0 = all */
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE, /* listen only: no probe requests from the Edge */
        /* scan_time left at 0: Bluetooth coexistence requires the default passive dwell. */
        .home_chan_dwell_time = 150,
    };
    if (esp_wifi_scan_start(&sc, false) != ESP_OK) return false;
    s_scan_in_flight = true;
    s_scan_issued = now_ms();
    return true;
}

static void start_scan(bool for_link)
{
    if (s_scan_busy) return;
    s_scan_started = now_ms();
    s_scan_for_link = for_link;
    if (link_in_use()) {
        /* Channel by channel, returning to the link channel in between. */
        s_sweep_ch = 1;
        s_sweep_next_at = s_scan_started;
        s_scan_busy = true;
    } else if (issue_scan(0)) {
        s_sweep_ch = 0;
        s_scan_busy = true;
    }
}

static uint8_t map_auth(wifi_auth_mode_t m)
{
    switch (m) {
    case WIFI_AUTH_OPEN: return EDGE_AUTH_OPEN;
    case WIFI_AUTH_WEP: return EDGE_AUTH_WEP;
    case WIFI_AUTH_WPA_PSK: return EDGE_AUTH_WPA;
    case WIFI_AUTH_WPA2_PSK:
    case WIFI_AUTH_WPA_WPA2_PSK: return EDGE_AUTH_WPA2;
    case WIFI_AUTH_WPA3_PSK: return EDGE_AUTH_WPA3;
    case WIFI_AUTH_WPA2_WPA3_PSK: return EDGE_AUTH_WPA2_WPA3;
    case WIFI_AUTH_WPA2_ENTERPRISE:
    case WIFI_AUTH_WPA3_ENTERPRISE:
    case WIFI_AUTH_WPA2_WPA3_ENTERPRISE: return EDGE_AUTH_ENTERPRISE;
    case WIFI_AUTH_OWE: return EDGE_AUTH_OWE;
    default: return EDGE_AUTH_UNKNOWN;
    }
}

static void collect_scan(void)
{
    uint16_t n = 24;
    wifi_ap_record_t *recs = calloc(n, sizeof(wifi_ap_record_t));
    if (!recs) {
        esp_wifi_clear_ap_list();
        return;
    }
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) n = 0;

    for (int i = 0; i < n; i++) {
        /* Scan lists are another view of the same access points the sniffer sees. */
        if (s_sniff_aps) {
            sense_ap_report_t r = {.channel = recs[i].primary, .rssi = recs[i].rssi,
                                   .auth = map_auth(recs[i].authmode), .flags = EDGE_APF_FROM_SCAN};
            memcpy(r.bssid, recs[i].bssid, 6);
            r.ssid_len = (uint8_t)strnlen((const char *)recs[i].ssid, 32);
            memcpy(r.ssid, recs[i].ssid, r.ssid_len);
            if (!r.ssid_len) r.flags |= EDGE_APF_HIDDEN;
            sense_feed_ap(&r);
        }
        near_update(recs[i].ssid, strnlen((const char *)recs[i].ssid, 32), recs[i].rssi, recs[i].primary,
                    map_auth(recs[i].authmode));
    }
    free(recs);
}

/* Known networks heard since the scan started, strongest first. */
static void build_candidates(void)
{
    edge_wifi_net_t known[EDGE_MAX_WIFI_NETS];
    int nk = edge_wifi_nets_get(known, EDGE_MAX_WIFI_NETS);
    s_ncand = 0;
    portENTER_CRITICAL(&s_near_mux);
    for (int i = 0; i < MAX_NEARBY; i++) {
        if (!s_near[i].ssid[0] || s_near[i].seen < s_scan_started) continue;
        for (int k = 0; k < nk && s_ncand < EDGE_MAX_WIFI_NETS; k++) {
            if (strcmp(known[k].ssid, s_near[i].ssid) == 0) {
                edge_strlcpy(s_cand[s_ncand].ssid, s_near[i].ssid, sizeof(s_cand[0].ssid));
                s_cand[s_ncand].channel = s_near[i].channel;
                s_cand[s_ncand].rssi = s_near[i].rssi;
                s_ncand++;
            }
        }
    }
    portEXIT_CRITICAL(&s_near_mux);
    for (int i = 1; i < s_ncand; i++) {
        for (int j = i; j > 0 && s_cand[j].rssi > s_cand[j - 1].rssi; j--) {
            cand_t t = s_cand[j];
            s_cand[j] = s_cand[j - 1];
            s_cand[j - 1] = t;
        }
    }
}

static int known_candidate(int nth)
{
    return nth < s_ncand ? nth : -1;
}

/* Drives full scans and channel sweeps; flags completion for the link logic. */
static void scan_service(void)
{
    int64_t t = now_ms();
    if (s_scan_in_flight && (s_scan_done || t - s_scan_issued > SCAN_STUCK_MS)) {
        s_scan_in_flight = false;
        bool done = s_scan_done;
        s_scan_done = false;
        if (done) {
            collect_scan();
        } else {
            esp_wifi_scan_stop();
        }
        if (s_sweep_ch) {
            s_sweep_ch++;
            s_sweep_next_at = t + SWEEP_HOME_MS;
        }
    }
    if (s_sweep_ch && !s_scan_in_flight && t >= s_sweep_next_at) {
        if (s_sweep_ch > 13 || s_state == EDGE_WIFI_STA_CONNECTING) {
            s_sweep_ch = 0; /* finished (or the link needs the radio) */
        } else if (!issue_scan(s_sweep_ch)) {
            s_sweep_ch++; /* skip a channel the driver refused */
        }
    }
    if (s_scan_busy && !s_scan_in_flight && !s_sweep_ch) {
        s_scan_busy = false;
        s_last_scan_done = t;
        if (s_scan_for_link) {
            s_scan_for_link = false;
            build_candidates();
            s_link_scan_complete = true;
        }
    }
}

static void set_ap_config(void)
{
    const edge_identity_t *id = edge_identity();
    wifi_config_t ap = {0};
    edge_strlcpy((char *)ap.ap.ssid, id->ap_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = (uint8_t)strlen(id->ap_ssid);
    edge_strlcpy((char *)ap.ap.password, id->ap_password, sizeof(ap.ap.password));
    ap.ap.channel = CONFIG_EDGE_AP_CHANNEL;
    ap.ap.max_connection = 3;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.pmf_cfg.capable = true;
    esp_wifi_set_config(WIFI_IF_AP, &ap);
}

static void go_off(void)
{
    esp_wifi_disconnect();
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_state = EDGE_WIFI_OFF;
    s_stations = 0;
    s_joined[0] = '\0';
    ESP_LOGI(TAG, "link off (Bluetooth only)");
}

static void go_ap(void)
{
    esp_wifi_disconnect();
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    set_ap_config();
    s_state = EDGE_WIFI_AP;
    s_cur_channel = CONFIG_EDGE_AP_CHANNEL;
    s_last_link_scan = now_ms();
    s_joined[0] = '\0';
    ESP_LOGI(TAG, "access point %s on channel %d", edge_identity()->ap_ssid, CONFIG_EDGE_AP_CHANNEL);
}

static void connect_to(int cand_idx)
{
    edge_wifi_net_t known[EDGE_MAX_WIFI_NETS];
    int nk = edge_wifi_nets_get(known, EDGE_MAX_WIFI_NETS);
    const edge_wifi_net_t *net = NULL;
    for (int k = 0; k < nk; k++) {
        if (strcmp(known[k].ssid, s_cand[cand_idx].ssid) == 0) net = &known[k];
    }
    if (!net) return;
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_stations = 0;
    wifi_config_t sta = {0};
    edge_strlcpy((char *)sta.sta.ssid, net->ssid, sizeof(sta.sta.ssid));
    edge_strlcpy((char *)sta.sta.password, net->password, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = net->password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    sta.sta.pmf_cfg.capable = true;
    sta.sta.channel = s_cand[cand_idx].channel;
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &sta);
    s_disconnected = false;
    s_got_ip = false;
    esp_wifi_connect();
    edge_strlcpy(s_joined, net->ssid, sizeof(s_joined));
    s_cur_channel = s_cand[cand_idx].channel;
    s_state = EDGE_WIFI_STA_CONNECTING;
    s_connect_started = now_ms();
    ESP_LOGI(TAG, "joining %s", net->ssid);
}

static void begin_search(void)
{
    if (s_state == EDGE_WIFI_AP || s_state == EDGE_WIFI_STA_CONNECTED) esp_wifi_disconnect();
    s_state = EDGE_WIFI_STA_SEARCHING;
    s_fails = 0;
    s_cand_idx = 0;
    start_scan(true);
}

/* After a failed attempt: next known network, else fall back. */
static void connect_failed(uint8_t link_mode)
{
    if (++s_fails < MAX_FAILS) {
        int idx = known_candidate(s_cand_idx);
        if (idx >= 0) {
            connect_to(idx);
            return;
        }
    }
    s_fails = 0;
    int idx = known_candidate(++s_cand_idx);
    if (idx >= 0) {
        connect_to(idx);
        return;
    }
    if (link_mode == EDGE_LINK_AUTO) {
        go_ap();
    } else {
        esp_wifi_disconnect();
        s_state = EDGE_WIFI_STA_WAITING;
        s_retry_at = now_ms() + STA_RETRY_MS;
    }
}

static void link_step(const edge_config_t *cfg)
{
    int64_t t = now_ms();

    if (s_reconfig) {
        s_reconfig = false;
        switch (cfg->link_mode) {
        case EDGE_LINK_OFF: go_off(); break;
        case EDGE_LINK_AP: go_ap(); break;
        default: begin_search(); break;
        }
    }

    scan_service();
    if (s_link_scan_complete) {
        s_link_scan_complete = false;
        {
            int idx = known_candidate(0);
            if (s_state == EDGE_WIFI_STA_SEARCHING) {
                if (idx >= 0) {
                    connect_to(idx);
                } else if (cfg->link_mode == EDGE_LINK_AUTO) {
                    go_ap();
                } else {
                    s_state = EDGE_WIFI_STA_WAITING;
                    s_retry_at = t + STA_RETRY_MS;
                }
            } else if (s_state == EDGE_WIFI_AP && cfg->link_mode == EDGE_LINK_AUTO && idx >= 0 && s_stations == 0) {
                s_fails = 0;
                s_cand_idx = 0;
                connect_to(idx);
            }
        }
    }

    switch (s_state) {
    case EDGE_WIFI_STA_CONNECTING:
        if (s_got_ip) {
            s_state = EDGE_WIFI_STA_CONNECTED;
            s_fails = 0;
            edge_time_start_sntp(); /* network time while joined to a network */
            ESP_LOGI(TAG, "joined %s, address %s", s_joined, s_ip);
        } else if (s_disconnected || t - s_connect_started > CONNECT_TIMEOUT_MS) {
            s_disconnected = false;
            connect_failed(cfg->link_mode);
        }
        break;
    case EDGE_WIFI_STA_CONNECTED:
        if (s_disconnected) {
            s_disconnected = false;
            s_state = EDGE_WIFI_STA_CONNECTING;
            s_connect_started = t;
            esp_wifi_connect();
        }
        break;
    case EDGE_WIFI_STA_WAITING:
        if (t >= s_retry_at) begin_search();
        break;
    case EDGE_WIFI_AP:
        if (cfg->link_mode == EDGE_LINK_AUTO && s_stations == 0 && !s_scan_busy && t - s_last_link_scan > AP_RESCAN_MS) {
            edge_wifi_net_t known[1];
            if (edge_wifi_nets_get(known, 1) > 0) start_scan(true);
            s_last_link_scan = t;
        }
        break;
    default:
        break;
    }
}

/* ---- Sniffer ------------------------------------------------------------------ */

static bool can_hop(void)
{
    if (s_scan_busy) return false;
    switch (s_state) {
    case EDGE_WIFI_OFF:
    case EDGE_WIFI_STA_WAITING:
        return true;
    case EDGE_WIFI_AP:
        return s_stations == 0;
    default:
        return false;
    }
}

static void sniff_step(void)
{
    bool want = s_sniff_aps || s_sniff_probes;
    if (want != s_promisc_on) {
        if (want) {
            wifi_promiscuous_filter_t flt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
            esp_wifi_set_promiscuous_filter(&flt);
            esp_wifi_set_promiscuous_rx_cb(promisc_cb);
            esp_wifi_set_promiscuous(true);
        } else {
            esp_wifi_set_promiscuous(false);
        }
        s_promisc_on = want;
    }
    if (!want) {
        s_hopping = false;
        return;
    }
    int64_t t = now_ms();
    bool hop = can_hop();
    if (hop && t - s_last_hop >= HOP_DWELL_MS) {
        s_last_hop = t;
        uint8_t ch;
        if (s_state == EDGE_WIFI_AP && (s_hop_step & 1) == 0) {
            ch = CONFIG_EDGE_AP_CHANNEL; /* keep the access point reachable */
        } else {
            ch = (uint8_t)(1 + (s_hop_step / (s_state == EDGE_WIFI_AP ? 2 : 1)) % 13);
        }
        s_hop_step++;
        if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK) s_cur_channel = ch;
    } else if (!hop && s_hopping && !s_scan_busy && s_stations == 0 && s_state == EDGE_WIFI_AP &&
               s_cur_channel != CONFIG_EDGE_AP_CHANNEL) {
        if (esp_wifi_set_channel(CONFIG_EDGE_AP_CHANNEL, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
            s_cur_channel = CONFIG_EDGE_AP_CHANNEL;
        }
    }
    s_hopping = hop;

    /* Fixed channel: complement the single-channel view with passive scans. */
    int64_t every = s_state == EDGE_WIFI_STA_CONNECTED ? SENSE_SCAN_STA_MS : SENSE_SCAN_MS;
    if (!hop && s_sniff_aps && !s_scan_busy && t - s_last_sense_scan > every &&
        ((s_state == EDGE_WIFI_AP && s_stations == 0) || s_state == EDGE_WIFI_STA_CONNECTED)) {
        s_last_sense_scan = t;
        start_scan(false);
    }
}

static void wifi_task(void *arg)
{
    edge_config_t cfg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TICK_MS));
        edge_config_get(&cfg);
        link_step(&cfg);
        if (s_scan_requested && !s_scan_busy) {
            s_scan_requested = false;
            start_scan(false);
        }
        sniff_step();
    }
}

/* ---- Public ------------------------------------------------------------------- */

esp_err_t edge_wifi_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    /* Hand out the Edge itself as DNS server on its access point (captive portal). */
    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_dns_info_t dns = {0};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    esp_netif_set_dns_info(s_ap_netif, ESP_NETIF_DNS_MAIN, &dns);
    uint8_t offer_dns = 0x02; /* OFFER_DNS */
    esp_netif_dhcps_option(s_ap_netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer_dns, sizeof(offer_dns));
    esp_netif_dhcps_start(s_ap_netif);

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    esp_wifi_set_storage(WIFI_STORAGE_RAM); /* credentials live in our own NVS namespace */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));

    /* Do not expose the factory (Espressif OUI) MACs: the station uses a random
     * per-device address, the access point a new random address every boot. */
    const edge_identity_t *id = edge_identity();
    memcpy(s_sta_mac, id->sta_mac, 6);
    esp_fill_random(s_ap_mac, 6);
    s_ap_mac[0] = (uint8_t)((s_ap_mac[0] & 0xfc) | 0x02);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    if (esp_wifi_set_mac(WIFI_IF_STA, s_sta_mac) != ESP_OK) ESP_LOGW(TAG, "could not set station MAC");
    if (esp_wifi_set_mac(WIFI_IF_AP, s_ap_mac) != ESP_OK) ESP_LOGW(TAG, "could not set AP MAC");
    set_ap_config();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    /* Channels 1-13. The Edge only listens and scans passively, so it never
     * transmits on 12-13 unless the owner adds a network that lives there. */
    wifi_country_t country = {.cc = "01", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    esp_wifi_set_country(&country);
    ESP_ERROR_CHECK(esp_wifi_start());

    if (mdns_init() == ESP_OK) {
        /* sotreus-xxxx.local, plus a _sotreus._tcp service the app can browse for. */
        char host[24], instance[32];
        snprintf(host, sizeof(host), "sotreus-%s", id->suffix);
        for (char *c = host; *c; c++) *c = (char)((*c >= 'A' && *c <= 'Z') ? *c + 32 : *c);
        snprintf(instance, sizeof(instance), "Sotreus Edge %s", id->suffix);
        mdns_hostname_set(host);
        mdns_instance_name_set(instance);
        mdns_txt_item_t txt[] = {
            {"vendor", "NetSepio LLC"},
            {"product", "Sotreus Edge"},
            {"url", "https://sotreus.com"},
            {"model", "heltec_wifi_lora_32_v3"},
            {"fw", esp_app_get_description()->version},
            {"api", "1"},
            {"path", "/"},
        };
        size_t ntxt = sizeof(txt) / sizeof(txt[0]);
        mdns_service_add(instance, "_http", "_tcp", 80, txt, ntxt);
        mdns_service_add(instance, "_sotreus", "_tcp", 80, txt, ntxt);
    }

    xTaskCreatePinnedToCore(wifi_task, "wifi_link", 6144, NULL, 5, &s_task, 0);
    return ESP_OK;
}

void edge_wifi_reconfigure(void)
{
    s_reconfig = true;
    if (s_task) xTaskNotifyGive(s_task);
}

void edge_wifi_sniff_set(bool aps, bool probes)
{
    s_sniff_aps = aps;
    s_sniff_probes = probes;
    if (s_task) xTaskNotifyGive(s_task);
}

void edge_wifi_request_scan(void)
{
    s_scan_requested = true;
    if (s_task) xTaskNotifyGive(s_task);
}

void edge_wifi_get_info(edge_wifi_info_t *o)
{
    memset(o, 0, sizeof(*o));
    o->state = s_state;
    o->stations = s_stations;
    o->channel = s_cur_channel;
    o->sniffing = s_promisc_on;
    o->hopping = s_hopping;
    if (s_state == EDGE_WIFI_AP) {
        edge_strlcpy(o->ssid, edge_identity()->ap_ssid, sizeof(o->ssid));
        edge_strlcpy(o->ip, "192.168.4.1", sizeof(o->ip));
    } else {
        edge_strlcpy(o->ssid, s_joined, sizeof(o->ssid));
        if (s_state == EDGE_WIFI_STA_CONNECTED) {
            edge_strlcpy(o->ip, s_ip, sizeof(o->ip));
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) o->rssi = ap.rssi;
        }
    }
}

cJSON *edge_wifi_scan_json(void)
{
    near_t copy[MAX_NEARBY];
    portENTER_CRITICAL(&s_near_mux);
    memcpy(copy, s_near, sizeof(copy));
    portEXIT_CRITICAL(&s_near_mux);
    /* strongest first */
    for (int i = 1; i < MAX_NEARBY; i++) {
        for (int j = i; j > 0 && copy[j].rssi > copy[j - 1].rssi; j--) {
            near_t t = copy[j];
            copy[j] = copy[j - 1];
            copy[j - 1] = t;
        }
    }
    edge_wifi_net_t known[EDGE_MAX_WIFI_NETS];
    int nk = edge_wifi_nets_get(known, EDGE_MAX_WIFI_NETS);
    int64_t t = now_ms();
    cJSON *a = cJSON_CreateArray();
    for (int i = 0; i < MAX_NEARBY; i++) {
        if (!copy[i].ssid[0] || t - copy[i].seen > NEARBY_TTL_MS) continue;
        cJSON *o = cJSON_CreateObject();
        char ssid[33 * 2];
        edge_sanitize_text(ssid, sizeof(ssid), (const uint8_t *)copy[i].ssid, strlen(copy[i].ssid));
        cJSON_AddStringToObject(o, "ssid", ssid);
        cJSON_AddNumberToObject(o, "rssi", copy[i].rssi);
        cJSON_AddNumberToObject(o, "channel", copy[i].channel);
        cJSON_AddStringToObject(o, "auth", edge_auth_name(copy[i].auth));
        cJSON_AddNumberToObject(o, "age_s", (double)((t - copy[i].seen) / 1000));
        bool k = false;
        for (int n = 0; n < nk; n++) k |= strcmp(known[n].ssid, copy[i].ssid) == 0;
        cJSON_AddBoolToObject(o, "known", k);
        cJSON_AddItemToArray(a, o);
    }
    return a;
}

bool edge_wifi_scanning(void)
{
    return s_scan_busy || s_scan_requested;
}
