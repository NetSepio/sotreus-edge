#include "mesh.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "aes/esp_aes.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "board.h"
#include "edge_app.h"
#include "edge_meshmod.h"
#include "edge_battery.h"
#include "edge_ble.h"
#include "edge_config.h"
#include "edge_events.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "edge_wifi.h"
#include "mesh_admin.h"
#include "mesh_crypto.h"
#include "mesh_mqtt.h"
#include "mesh_pb_ext.h"
#include "mesh_radio.h"
#include "mesh_sfdb.h"
#include "sx1262.h"

static const char *TAG = "mesh";

#define TXQ_LEN         16
#define SEEN_LEN        128
#define SEEN_TTL_MS     (10 * 60 * 1000)
#define MAX_NODES       64
#define MAX_MSGS        32
#define MAX_WAYPOINTS   16
#define MAX_PENDING     8
#define MAX_TEXT        200
#define ACK_RETRIES     3
#define MAX_NB          6
#define MAX_SF_ROUTERS  4
#define PAX_SLOTS       256
#define SAVE_EVERY_MS   (15 * 60 * 1000)

#define REQ_START   0x01
#define REQ_STOP    0x02
#define REQ_RELOAD  0x04
#define REQ_SAVE    0x08

/* node flags */
#define NF_USER     0x01
#define NF_PKC      0x02
#define NF_FAV      0x04
#define NF_IGNORE   0x08
#define NF_POS      0x10
#define NF_DEV      0x20
#define NF_ENV      0x40

typedef struct {
    bool used;
    bool is_relay;
    bool track;
    bool late;           /* ROUTER_LATE relay: not cancelled when others relay first */
    uint8_t len;
    uint8_t tries;
    uint32_t from, id, to;
    int64_t not_before;
    uint8_t buf[MESH_MAX_PACKET];
} tx_item_t;

typedef struct {
    bool used;
    bool broadcast;
    uint32_t id, to;
    int64_t deadline;
    uint8_t tries;
    uint8_t len;
    uint8_t buf[MESH_MAX_PACKET];
} pending_t;

typedef struct {
    uint32_t from, id;
    int64_t at;
    uint32_t to;
    bool relayed;
} seen_t;

typedef struct {
    uint32_t num;
    uint8_t flags;
    uint8_t role;
    uint8_t hw_model;
    int8_t hops_away;    /* -1 unknown */
    char long_name[40];
    char short_name[8];
    uint8_t pubkey[32];
    int16_t rssi;
    int8_t snr_x4;
    uint8_t next_hop;    /* last byte of the neighbour that reaches it, 0 = flood */
    uint32_t last_heard_s;   /* unix seconds, 0 if unknown */
    int32_t lat_i, lon_i, alt;
    uint32_t pos_time;
    uint8_t pos_bits;
    uint8_t battery;
    uint16_t voltage_mv;
    uint16_t ch_util_x10, air_util_x10;
    uint32_t uptime_s;
    int16_t temp_x10;
    uint16_t hum_x10, press_x10;
    uint8_t n_nb;
    struct {
        uint32_t num;
        int8_t snr_x4;
    } nb[MAX_NB];
    /* runtime only */
    int64_t last_heard_up;
    int64_t last_nodeinfo_reply;
    bool key_ok;
    uint8_t shared[32];
} node_t;

typedef struct {
    bool used;
    bool outgoing;
    bool pki;
    uint32_t id, from, to, reply_id, emoji;
    uint8_t channel;
    int64_t at;
    uint32_t at_s;
    int16_t rssi;
    int8_t snr_x4;
    char status[12];
    char text[MAX_TEXT + 34];
} msg_t;

typedef struct {
    bool used;
    uint32_t from;
    uint32_t at_s;
    mt_waypoint_t w;
} wp_t;

typedef struct {
    char name[12];
    uint8_t key[32];
    size_t key_len;
    uint8_t hash;
    uint8_t precision;
} chan_t;

/* ---- State ---------------------------------------------------------------------- */

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static volatile uint32_t s_req;
static bool s_radio_ok;
static volatile bool s_running;

static edge_config_t s_cfg;
static edge_mesh_mod_t s_mod;
static const mesh_region_t *s_region;
static const mesh_preset_t *s_preset;
static float s_freq;
static chan_t s_ch[EDGE_MESH_MAX_CHANNELS];
static int s_nch;
static uint32_t s_me;
static uint8_t s_priv[32], s_pub[32];
static bool s_have_keys;

/* allocated only while Mesh mode runs */
static tx_item_t *s_txq;
static pending_t *s_pending;
static seen_t *s_seen;
static node_t *s_nodes;
static msg_t *s_msgs;
static wp_t *s_wps;
static int s_seen_next, s_msg_next;
static bool s_dirty;

static bool s_tx_busy;
static tx_item_t s_tx_cur;
static int64_t s_tx_started;
static uint32_t s_tx_airtime;
static int64_t s_rx_activity;

static uint32_t s_air_tx_min[60];
static int64_t s_air_min_start;
static int s_air_min_idx;
static uint32_t s_util_10s[6];
static int64_t s_util_start;
static int s_util_idx;

static int64_t s_next_nodeinfo, s_next_position, s_next_telemetry, s_next_neighbor, s_next_range, s_next_pax,
    s_next_save, s_next_lost, s_pax_reassert;
static uint32_t s_range_seq;

static bool s_client_pos;
static int32_t s_client_lat, s_client_lon, s_client_alt;

static struct {
    uint32_t num;
    int64_t at;
} s_sf[MAX_SF_ROUTERS];

static portMUX_TYPE s_pax_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_pax_on;
static uint32_t s_pax_ble[PAX_SLOTS], s_pax_wifi[PAX_SLOTS];
static int s_pax_nb, s_pax_nw;

static volatile int64_t s_notify_until;
static bool s_serial_on;
static char s_serial_line[MAX_TEXT + 1];
static size_t s_serial_len;
static bool s_detect_on;
static int s_detect_last;
static int64_t s_detect_sent;

static struct {
    uint32_t rx_ok, rx_bad, rx_undecoded, rx_dupe, rx_pki, tx_ok, relayed, relay_cancelled, tx_dropped,
        preambles, headers, ignored;
} s_st;

static const char *ROLE_NAMES[] = {"CLIENT", "CLIENT_MUTE", "ROUTER", "ROUTER_CLIENT", "REPEATER", "TRACKER",
                                   "SENSOR", "TAK", "CLIENT_HIDDEN", "LOST_AND_FOUND", "TAK_TRACKER", "ROUTER_LATE"};

/* ---- Helpers ---------------------------------------------------------------------- */

static int64_t now_ms(void)
{
    return edge_uptime_ms();
}

static uint32_t now_unix_s(void)
{
    int64_t u = edge_time_to_unix_ms(now_ms());
    return u > 0 ? (uint32_t)(u / 1000) : 0;
}

static uint32_t rnd(uint32_t n)
{
    return n ? esp_random() % n : 0;
}

static void node_id_str(uint32_t n, char out[12])
{
    if (n == MT_BROADCAST) {
        strcpy(out, "broadcast");
    } else {
        snprintf(out, 12, "!%08lx", (unsigned long)n);
    }
}

static const char *role_name(uint8_t r)
{
    return r < sizeof(ROLE_NAMES) / sizeof(ROLE_NAMES[0]) ? ROLE_NAMES[r] : "CLIENT";
}

static bool is_ignored(uint32_t num)
{
    for (int i = 0; i < s_cfg.mx.n_ignore; i++) {
        if (s_cfg.mx.ignore[i] == num) return true;
    }
    return false;
}

static void channel_crypt(const chan_t *ch, uint32_t from, uint32_t id, uint8_t *data, size_t n)
{
    if (ch->key_len == 0 || n == 0) return;
    uint8_t nonce[16] = {0};
    uint64_t pid = id;
    memcpy(nonce, &pid, 8);
    memcpy(nonce + 8, &from, 4);
    uint8_t stream[16];
    size_t off = 0;
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    esp_aes_setkey(&ctx, ch->key, ch->key_len * 8);
    esp_aes_crypt_ctr(&ctx, n, &off, nonce, stream, data, data);
    esp_aes_free(&ctx);
}

/* ---- Seen cache ------------------------------------------------------------------- */

static seen_t *seen_find(uint32_t from, uint32_t id)
{
    int64_t t = now_ms();
    for (int i = 0; i < SEEN_LEN; i++) {
        seen_t *s = &s_seen[i];
        if (s->at && s->from == from && s->id == id && t - s->at < SEEN_TTL_MS) return s;
    }
    return NULL;
}

static seen_t *seen_add(uint32_t from, uint32_t id, uint32_t to)
{
    seen_t *s = &s_seen[s_seen_next];
    s_seen_next = (s_seen_next + 1) % SEEN_LEN;
    *s = (seen_t){.from = from, .id = id, .at = now_ms(), .to = to};
    return s;
}

/* ---- Node database ---------------------------------------------------------------- */

static node_t *node_get(uint32_t num, bool create)
{
    if (!s_nodes || num == 0 || num == MT_BROADCAST) return NULL;
    node_t *victim = NULL;
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].num == num) return &s_nodes[i];
    }
    if (!create) return NULL;
    for (int i = 0; i < MAX_NODES; i++) {
        node_t *n = &s_nodes[i];
        if (n->num == 0) {
            victim = n;
            break;
        }
        if (n->flags & NF_FAV) continue; /* favourites are never evicted */
        if (!victim || n->last_heard_up < victim->last_heard_up) victim = n;
    }
    if (!victim) return NULL;
    memset(victim, 0, sizeof(*victim));
    victim->num = num;
    victim->hops_away = -1;
    victim->last_heard_up = now_ms();
    s_dirty = true;
    return victim;
}

static int node_count(void)
{
    int n = 0;
    for (int i = 0; s_nodes && i < MAX_NODES; i++) n += s_nodes[i].num != 0;
    return n;
}

static bool node_key(node_t *n)
{
    if (!n || !(n->flags & NF_PKC) || !s_have_keys) return false;
    if (!n->key_ok) {
        mesh_pki_shared_key(n->shared, s_priv, n->pubkey);
        n->key_ok = true;
    }
    return true;
}

static cJSON *node_json(const node_t *n)
{
    cJSON *o = cJSON_CreateObject();
    char id[12];
    node_id_str(n->num, id);
    cJSON_AddStringToObject(o, "id", id);
    cJSON_AddNumberToObject(o, "num", n->num);
    if (n->flags & NF_USER) {
        char s[sizeof(n->long_name)];
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)n->long_name, strlen(n->long_name));
        cJSON_AddStringToObject(o, "long_name", s);
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)n->short_name, strlen(n->short_name));
        cJSON_AddStringToObject(o, "short_name", s);
        cJSON_AddNumberToObject(o, "hw_model", n->hw_model);
        cJSON_AddStringToObject(o, "role", role_name(n->role));
    }
    cJSON_AddBoolToObject(o, "has_public_key", (n->flags & NF_PKC) != 0);
    if (n->flags & NF_PKC) {
        char b64[48];
        edge_b64_encode(n->pubkey, 32, b64, sizeof(b64));
        cJSON_AddStringToObject(o, "public_key", b64);
    }
    cJSON_AddBoolToObject(o, "favorite", (n->flags & NF_FAV) != 0);
    cJSON_AddBoolToObject(o, "ignored", (n->flags & NF_IGNORE) != 0);
    if (n->last_heard_up > 0) cJSON_AddNumberToObject(o, "last_heard_ago_s", (double)((now_ms() - n->last_heard_up) / 1000));
    if (n->last_heard_s) cJSON_AddNumberToObject(o, "last_heard_unix_s", n->last_heard_s);
    cJSON_AddNumberToObject(o, "rssi", n->rssi);
    cJSON_AddNumberToObject(o, "snr", n->snr_x4 / 4.0);
    if (n->hops_away >= 0) cJSON_AddNumberToObject(o, "hops_away", n->hops_away);
    if (n->next_hop) cJSON_AddNumberToObject(o, "next_hop", n->next_hop);
    if (n->flags & NF_POS) {
        cJSON *p = cJSON_AddObjectToObject(o, "reported_position");
        cJSON_AddNumberToObject(p, "lat", n->lat_i / 1e7);
        cJSON_AddNumberToObject(p, "lon", n->lon_i / 1e7);
        cJSON_AddNumberToObject(p, "alt_m", n->alt);
        if (n->pos_bits) cJSON_AddNumberToObject(p, "precision_bits", n->pos_bits);
        if (n->pos_time) cJSON_AddNumberToObject(p, "reported_unix_s", n->pos_time);
        cJSON_AddStringToObject(p, "provenance", "NETWORK");
    }
    if (n->flags & NF_DEV) {
        cJSON *d = cJSON_AddObjectToObject(o, "device");
        if (n->battery) cJSON_AddNumberToObject(d, "battery_level", n->battery);
        if (n->voltage_mv) cJSON_AddNumberToObject(d, "voltage", n->voltage_mv / 1000.0);
        cJSON_AddNumberToObject(d, "channel_util", n->ch_util_x10 / 10.0);
        cJSON_AddNumberToObject(d, "air_util_tx", n->air_util_x10 / 10.0);
        if (n->uptime_s) cJSON_AddNumberToObject(d, "uptime_s", n->uptime_s);
    }
    if (n->flags & NF_ENV) {
        cJSON *e = cJSON_AddObjectToObject(o, "environment");
        cJSON_AddNumberToObject(e, "temperature_c", n->temp_x10 / 10.0);
        cJSON_AddNumberToObject(e, "humidity_pct", n->hum_x10 / 10.0);
        cJSON_AddNumberToObject(e, "pressure_hpa", n->press_x10 / 10.0);
    }
    if (n->n_nb) {
        cJSON *a = cJSON_AddArrayToObject(o, "neighbors");
        for (int i = 0; i < n->n_nb; i++) {
            cJSON *x = cJSON_CreateObject();
            char nid[12];
            node_id_str(n->nb[i].num, nid);
            cJSON_AddStringToObject(x, "id", nid);
            cJSON_AddNumberToObject(x, "snr", n->nb[i].snr_x4 / 4.0);
            cJSON_AddItemToArray(a, x);
        }
    }
    return o;
}

static void publish_node(const node_t *n)
{
    if (!edge_events_has_listeners()) return;
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_node");
    cJSON_AddItemToObject(ev, "node", node_json(n));
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

/* ---- Messages ----------------------------------------------------------------------- */

static msg_t *msg_add(void)
{
    msg_t *m = &s_msgs[s_msg_next];
    s_msg_next = (s_msg_next + 1) % MAX_MSGS;
    memset(m, 0, sizeof(*m));
    m->used = true;
    m->at = now_ms();
    m->at_s = now_unix_s();
    s_dirty = true;
    return m;
}

static msg_t *msg_find_out(uint32_t id)
{
    for (int i = 0; s_msgs && i < MAX_MSGS; i++) {
        if (s_msgs[i].used && s_msgs[i].outgoing && s_msgs[i].id == id) return &s_msgs[i];
    }
    return NULL;
}

static cJSON *msg_json(const msg_t *m)
{
    cJSON *o = cJSON_CreateObject();
    char id[12];
    cJSON_AddNumberToObject(o, "id", m->id);
    node_id_str(m->from, id);
    cJSON_AddStringToObject(o, "from", id);
    node_id_str(m->to, id);
    cJSON_AddStringToObject(o, "to", id);
    cJSON_AddNumberToObject(o, "channel", m->channel);
    cJSON_AddStringToObject(o, "dir", m->outgoing ? "sent" : "received");
    cJSON_AddStringToObject(o, "provenance", "NETWORK");
    cJSON_AddBoolToObject(o, "pki", m->pki);
    if (m->at > 0) cJSON_AddNumberToObject(o, "ago_s", (double)((now_ms() - m->at) / 1000));
    if (m->at_s) cJSON_AddNumberToObject(o, "unix_s", m->at_s);
    if (m->reply_id) cJSON_AddNumberToObject(o, "reply_id", m->reply_id);
    if (m->emoji) cJSON_AddTrueToObject(o, "reaction");
    if (m->outgoing) {
        cJSON_AddStringToObject(o, "status", m->status);
    } else {
        cJSON_AddNumberToObject(o, "rssi", m->rssi);
        cJSON_AddNumberToObject(o, "snr", m->snr_x4 / 4.0);
    }
    cJSON_AddStringToObject(o, "text", m->text);
    node_t *n = node_get(m->outgoing ? m->to : m->from, false);
    if (n && (n->flags & NF_USER)) cJSON_AddStringToObject(o, m->outgoing ? "to_name" : "from_name", n->short_name);
    return o;
}

static void publish_tx_status(uint32_t id, const char *status, int reason)
{
    msg_t *m = msg_find_out(id);
    if (m) {
        edge_strlcpy(m->status, status, sizeof(m->status));
        s_dirty = true;
    }
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_tx");
    cJSON_AddNumberToObject(ev, "id", id);
    cJSON_AddStringToObject(ev, "status", status);
    if (reason >= 0) cJSON_AddNumberToObject(ev, "reason", reason);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void log_text(uint32_t from, uint32_t to, uint32_t id, uint8_t ch, uint8_t hops, int16_t rssi, int8_t snr_x4,
                     bool outgoing, const char *text, size_t len)
{
    edge_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.type = EDGE_REC_MESH_TEXT;
    rec.first_ms = rec.last_ms = now_ms();
    edge_body_mesh_text_t *b = (edge_body_mesh_text_t *)rec.body;
    b->from = from;
    b->to = to;
    b->packet_id = id;
    b->channel = ch;
    b->hops = hops;
    b->rssi = (int8_t)(rssi < -128 ? -128 : rssi > 127 ? 127 : rssi);
    b->snr_x4 = snr_x4;
    b->dir = outgoing ? 1 : 0;
    if (len > EDGE_MESH_TEXT_MAX) {
        len = EDGE_MESH_TEXT_MAX;
        rec.flags |= EDGE_RECF_TRUNCATED;
    }
    memcpy(b->text, text, len);
    b->text_len = (uint8_t)len;
    rec.body_len = sizeof(*b);
    edge_log_append(&rec);
}

/* ---- Airtime, duty cycle and channel utilisation ------------------------------------ */

static void air_roll(void)
{
    int64_t t = now_ms();
    if (!s_air_min_start) s_air_min_start = t;
    while (t - s_air_min_start >= 60000) {
        s_air_min_start += 60000;
        s_air_min_idx = (s_air_min_idx + 1) % 60;
        s_air_tx_min[s_air_min_idx] = 0;
    }
    if (!s_util_start) s_util_start = t;
    while (t - s_util_start >= 10000) {
        s_util_start += 10000;
        s_util_idx = (s_util_idx + 1) % 6;
        s_util_10s[s_util_idx] = 0;
    }
}

static void air_add(uint32_t ms, bool tx)
{
    air_roll();
    if (tx) s_air_tx_min[s_air_min_idx] += ms;
    s_util_10s[s_util_idx] += ms;
}

static uint32_t air_tx_hour(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 60; i++) sum += s_air_tx_min[i];
    return sum;
}

static float channel_util_pct(void)
{
    air_roll();
    uint32_t sum = 0;
    for (int i = 0; i < 6; i++) sum += s_util_10s[i];
    return sum / 600.0f;
}

static float air_util_tx_pct(void)
{
    air_roll();
    return air_tx_hour() / 36000.0f;
}

static bool duty_allows(uint32_t airtime)
{
    if (!s_region || s_region->duty_cycle >= 100 || s_cfg.mx.override_duty_cycle) return true;
    air_roll();
    return air_tx_hour() + airtime <= (uint32_t)s_region->duty_cycle * 36000u;
}

/* Meshtastic "polite" rules for our own periodic broadcasts. */
static bool polite_allows(void)
{
    if (channel_util_pct() > 25.0f) return false;
    if (s_region && s_region->duty_cycle < 100 && !s_cfg.mx.override_duty_cycle &&
        air_util_tx_pct() > s_region->duty_cycle * 0.5f) {
        return false;
    }
    return true;
}

/* ---- Transmit queue ----------------------------------------------------------------- */

static tx_item_t *txq_alloc(void)
{
    for (int i = 0; i < TXQ_LEN; i++) {
        if (!s_txq[i].used) {
            memset(&s_txq[i], 0, sizeof(s_txq[i]));
            s_txq[i].used = true;
            return &s_txq[i];
        }
    }
    s_st.tx_dropped++;
    return NULL;
}

static bool txq_cancel_relay(uint32_t from, uint32_t id)
{
    for (int i = 0; i < TXQ_LEN; i++) {
        tx_item_t *t = &s_txq[i];
        if (t->used && t->is_relay && !t->late && t->from == from && t->id == id) {
            t->used = false;
            return true;
        }
    }
    return false;
}

static uint32_t new_packet_id(void)
{
    uint32_t id;
    do {
        id = esp_random();
    } while (id == 0);
    return id;
}

static void mqtt_uplink(const uint8_t *frame, size_t len, int ch, bool pki, const mt_data_t *d, bool from_us, int16_t rssi,
                        float snr);

/* Encrypts and queues a packet originated here. Caller holds s_lock. */
static esp_err_t send_data(uint32_t to, int ch_idx, const mt_data_t *d_in, bool want_ack, bool track, uint8_t hop_limit,
                           uint32_t *out_id, bool *used_pki)
{
    if (!s_running || !s_region || !s_txq) return ESP_ERR_INVALID_STATE;
    if (!s_cfg.mx.tx_enabled) return ESP_ERR_NOT_ALLOWED;
    if (ch_idx < 0 || ch_idx >= s_nch) return ESP_ERR_INVALID_ARG;
    tx_item_t *t = txq_alloc();
    if (!t) return ESP_ERR_NO_MEM;

    mt_data_t d = *d_in;
    if (s_cfg.mx.ok_to_mqtt) {
        d.has_bitfield = true;
        d.bitfield |= 1;
    }
    uint8_t plain[MESH_MAX_PAYLOAD];
    size_t n = mt_data_encode(&d, plain, sizeof(plain));
    if (n == 0) {
        t->used = false;
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t id = new_packet_id();
    node_t *peer = to != MT_BROADCAST ? node_get(to, false) : NULL;
    /* Direct messages use the recipient's public key when we know it (Meshtastic 2.5+). */
    bool pki = !s_cfg.mx.licensed && peer && node_key(peer) && n + MESH_PKI_OVERHEAD <= MESH_MAX_PAYLOAD &&
               d.portnum != MT_PORT_TRACEROUTE && d.portnum != MT_PORT_NODEINFO;
    mt_header_t h = {
        .to = to,
        .from = s_me,
        .id = id,
        .flags = mt_make_flags(hop_limit, hop_limit, want_ack),
        .channel = pki ? 0 : s_ch[ch_idx].hash,
        .next_hop = peer ? peer->next_hop : 0,
        .relay_node = (uint8_t)(s_me & 0xff),
    };
    mt_header_write(&h, t->buf);
    size_t plen;
    if (pki) {
        mesh_pki_encrypt(peer->shared, s_me, id, esp_random(), plain, n, t->buf + MESH_HEADER_LEN);
        plen = n + MESH_PKI_OVERHEAD;
    } else {
        memcpy(t->buf + MESH_HEADER_LEN, plain, n);
        channel_crypt(&s_ch[ch_idx], s_me, id, t->buf + MESH_HEADER_LEN, n);
        plen = n;
    }
    t->len = (uint8_t)(MESH_HEADER_LEN + plen);
    t->from = s_me;
    t->id = id;
    t->to = to;
    t->track = track;
    t->not_before = now_ms() + rnd(8) * mesh_slot_ms(s_preset);
    seen_add(s_me, id, to);
    if (out_id) *out_id = id;
    if (used_pki) *used_pki = pki;
    mqtt_uplink(t->buf, t->len, pki ? -1 : ch_idx, pki, &d, true, 0, 0);
    if (s_task) xTaskNotifyGive(s_task);
    return ESP_OK;
}

/* Re-sends a stored packet under its original sender and id (store & forward replay). */
static esp_err_t send_data_as(uint32_t from, uint32_t id, uint32_t to, int ch_idx, const mt_data_t *d)
{
    if (!s_running || !s_region || !s_txq || !s_cfg.mx.tx_enabled) return ESP_ERR_INVALID_STATE;
    if (ch_idx < 0 || ch_idx >= s_nch) return ESP_ERR_INVALID_ARG;
    uint8_t plain[MESH_MAX_PAYLOAD];
    size_t n = mt_data_encode(d, plain, sizeof(plain));
    if (n == 0) return ESP_ERR_INVALID_SIZE;
    tx_item_t *t = txq_alloc();
    if (!t) return ESP_ERR_NO_MEM;
    node_t *peer = node_get(to, false);
    mt_header_t h = {.to = to, .from = from, .id = id, .flags = mt_make_flags(s_cfg.hop_limit, s_cfg.hop_limit, false),
                     .channel = s_ch[ch_idx].hash, .next_hop = peer ? peer->next_hop : 0, .relay_node = (uint8_t)(s_me & 0xff)};
    mt_header_write(&h, t->buf);
    memcpy(t->buf + MESH_HEADER_LEN, plain, n);
    channel_crypt(&s_ch[ch_idx], from, id, t->buf + MESH_HEADER_LEN, n);
    t->len = (uint8_t)(MESH_HEADER_LEN + n);
    t->from = from;
    t->id = id;
    t->to = to;
    t->not_before = now_ms() + rnd(8) * mesh_slot_ms(s_preset);
    if (s_task) xTaskNotifyGive(s_task);
    return ESP_OK;
}

static esp_err_t send_port(uint32_t to, int ch, uint32_t port, const uint8_t *payload, size_t len, bool want_response,
                           uint32_t request_id, uint8_t hop_limit)
{
    mt_data_t d = {.portnum = port, .payload = payload, .payload_len = len, .want_response = want_response,
                   .request_id = request_id};
    return send_data(to, ch, &d, false, false, hop_limit, NULL, NULL);
}

static void send_routing(uint32_t to, uint32_t req_id, int ch, uint32_t error)
{
    uint8_t rp[8];
    size_t rl = mt_routing_encode_error(error, rp, sizeof(rp));
    send_port(to, ch, MT_PORT_ROUTING, rp, rl, false, req_id, s_cfg.hop_limit);
}

/* ---- Own broadcasts ------------------------------------------------------------------ */

static void send_nodeinfo(uint32_t to, bool want_response)
{
    mt_user_t u = {.hw_model = MT_HW_HELTEC_V3, .role = s_cfg.mx.role, .is_licensed = s_cfg.mx.licensed};
    snprintf(u.id, sizeof(u.id), "!%08lx", (unsigned long)s_me);
    edge_strlcpy(u.long_name, s_cfg.long_name, sizeof(u.long_name));
    edge_strlcpy(u.short_name, s_cfg.short_name, sizeof(u.short_name));
    if (s_have_keys && !s_cfg.mx.licensed) {
        u.has_public_key = true;
        memcpy(u.public_key, s_pub, 32);
    }
    uint8_t ub[128];
    size_t ul = mt_user_encode(&u, ub, sizeof(ub));
    send_port(to, 0, MT_PORT_NODEINFO, ub, ul, want_response, 0, s_cfg.hop_limit);
}

static bool own_position(int32_t *lat, int32_t *lon, int32_t *alt)
{
    if (s_cfg.mx.pos_mode == EDGE_POS_FIXED && (s_cfg.mx.pos_lat_i || s_cfg.mx.pos_lon_i)) {
        *lat = s_cfg.mx.pos_lat_i;
        *lon = s_cfg.mx.pos_lon_i;
        *alt = s_cfg.mx.pos_alt;
        return true;
    }
    if (s_cfg.mx.pos_mode == EDGE_POS_CLIENT && s_client_pos) {
        *lat = s_client_lat;
        *lon = s_client_lon;
        *alt = s_client_alt;
        return true;
    }
    return false;
}

/* Position on channel ch with that channel's precision; an empty position when not shared. */
static void send_position(uint32_t to, int ch, bool want_response, uint32_t request_id)
{
    mt_position_t p = {0};
    int32_t lat, lon, alt;
    uint8_t bits = s_ch[ch].precision;
    if (own_position(&lat, &lon, &alt) && bits) {
        p.has_lat = p.has_lon = p.has_alt = true;
        p.lat_i = mt_position_truncate(lat, bits);
        p.lon_i = mt_position_truncate(lon, bits);
        p.alt = alt;
        p.location_source = 1; /* manual */
        p.precision_bits = bits;
        p.time = now_unix_s();
    }
    uint8_t pb[64];
    size_t pl = mt_position_encode(&p, pb, sizeof(pb));
    send_port(to, ch, MT_PORT_POSITION, pb, pl, want_response, request_id, s_cfg.hop_limit);
}

static void send_telemetry(uint32_t to, uint32_t request_id)
{
    edge_battery_t b;
    edge_battery_get(&b);
    mt_telemetry_t t = {.time = now_unix_s(), .has_battery = true, .battery_level = b.present ? (uint32_t)b.percent : 101,
                        .has_ch_util = true, .channel_utilization = channel_util_pct(),
                        .has_air_util = true, .air_util_tx = air_util_tx_pct(),
                        .has_uptime = true, .uptime_seconds = (uint32_t)(now_ms() / 1000)};
    if (b.present) {
        t.has_voltage = true;
        t.voltage = b.mv / 1000.0f;
    }
    uint8_t tb[64];
    size_t tl = mt_telemetry_encode(&t, tb, sizeof(tb));
    send_port(to, 0, MT_PORT_TELEMETRY, tb, tl, false, request_id, s_cfg.hop_limit);
}

static void send_neighbor_info(void)
{
    mt_neighborinfo_t ni = {.node_id = s_me, .last_sent_by_id = s_me, .interval_secs = s_cfg.mx.neighbor_interval_s};
    int64_t window = (int64_t)s_cfg.mx.neighbor_interval_s * 2000;
    for (int i = 0; i < MAX_NODES && ni.n < MT_MAX_NEIGHBORS; i++) {
        node_t *n = &s_nodes[i];
        if (!n->num || n->hops_away != 0 || n->last_heard_up <= 0 || now_ms() - n->last_heard_up > window) continue;
        mt_neighbor_t *x = &ni.nb[ni.n++];
        x->node_id = n->num;
        x->snr = n->snr_x4 / 4.0f;
    }
    uint8_t buf[200];
    size_t l = mt_neighborinfo_encode(&ni, buf, sizeof(buf));
    if (l) send_port(MT_BROADCAST, 0, MT_PORT_NEIGHBORINFO, buf, l, false, 0, 0); /* zero hop, like Meshtastic */
}


/* ==== Mesh modules: MQTT, store & forward server, remote admin, remote hardware ========== */

#define FW_COMPAT "2.5.0.5e0e" /* Meshtastic feature level reported to other nodes' apps */

static void process_rx(uint8_t *buf, size_t len, int16_t rssi, float snr);
static void store_text(const mt_header_t *h, int ch, bool pki, const mt_data_t *d, const uint8_t *text, size_t len,
                       int16_t rssi, float snr, const char *prefix);

static bool s_rx_from_mqtt;
static bool s_mqtt_restart;
static bool s_mqtt_public;
static char s_mqtt_root[48];
static int64_t s_mqtt_check, s_map_next;
static uint8_t s_dl_buf[MQTT_RX_MAX];

static int64_t s_reload_at, s_reboot_at, s_nodeinfo_at;
static bool s_wifi_reconf;
static uint8_t s_passkey[8];
static int64_t s_passkey_at = -1;
static uint32_t s_admin_seen, s_admin_refused;

static uint64_t s_rhw_out, s_rhw_watch, s_rhw_last;
static uint32_t s_rhw_watcher;
static int s_rhw_ch;
static int64_t s_rhw_sent;

static bool s_sfs_on;
static struct {
    uint32_t num;
    int ch;
    uint32_t seqs[100];
    int n, idx;
    int64_t next_at;
} s_sfs;
static struct {
    uint32_t num, last_s;
} s_sfs_last[8];
static uint32_t s_sfs_requests, s_sfs_hist_requests;
static int64_t s_sfs_hb_next;
static uint32_t s_audio_frames;

static bool wifi_up(edge_wifi_info_t *wi)
{
    edge_wifi_get_info(wi);
    return wi->state == EDGE_WIFI_STA_CONNECTED && wi->ip[0];
}

static void gateway_id(char out[16])
{
    snprintf(out, 16, "!%08lx", (unsigned long)s_me);
}

/* ---- MQTT ---------------------------------------------------------------------------- */

static void mqtt_json_uplink(const mt_header_t *h, int ch, const mt_data_t *d, int16_t rssi, float snr)
{
    cJSON *o = cJSON_CreateObject();
    cJSON *pl = NULL;
    const char *type = NULL;
    switch (d->portnum) {
    case MT_PORT_TEXT: {
        char t[MAX_TEXT + 34];
        edge_sanitize_text(t, sizeof(t), d->payload, d->payload_len);
        pl = cJSON_CreateObject();
        cJSON_AddStringToObject(pl, "text", t);
        type = "text";
        break;
    }
    case MT_PORT_POSITION: {
        mt_position_t p;
        if (!mt_position_decode(d->payload, d->payload_len, &p)) break;
        pl = cJSON_CreateObject();
        if (p.has_lat) cJSON_AddNumberToObject(pl, "latitude_i", p.lat_i);
        if (p.has_lon) cJSON_AddNumberToObject(pl, "longitude_i", p.lon_i);
        if (p.has_alt) cJSON_AddNumberToObject(pl, "altitude", p.alt);
        if (p.precision_bits) cJSON_AddNumberToObject(pl, "precision_bits", p.precision_bits);
        if (p.time) cJSON_AddNumberToObject(pl, "time", p.time);
        type = "position";
        break;
    }
    case MT_PORT_NODEINFO: {
        mt_user_t u;
        if (!mt_user_decode(d->payload, d->payload_len, &u)) break;
        pl = cJSON_CreateObject();
        cJSON_AddStringToObject(pl, "id", u.id);
        cJSON_AddStringToObject(pl, "longname", u.long_name);
        cJSON_AddStringToObject(pl, "shortname", u.short_name);
        cJSON_AddNumberToObject(pl, "hardware", u.hw_model);
        cJSON_AddNumberToObject(pl, "role", u.role);
        type = "nodeinfo";
        break;
    }
    case MT_PORT_TELEMETRY: {
        mt_telemetry_t t;
        if (!mt_telemetry_decode(d->payload, d->payload_len, &t)) break;
        pl = cJSON_CreateObject();
        if (t.has_battery) cJSON_AddNumberToObject(pl, "battery_level", t.battery_level);
        if (t.has_voltage) cJSON_AddNumberToObject(pl, "voltage", t.voltage);
        if (t.has_ch_util) cJSON_AddNumberToObject(pl, "channel_utilization", t.channel_utilization);
        if (t.has_air_util) cJSON_AddNumberToObject(pl, "air_util_tx", t.air_util_tx);
        if (t.has_uptime) cJSON_AddNumberToObject(pl, "uptime_seconds", t.uptime_seconds);
        if (t.has_temp) cJSON_AddNumberToObject(pl, "temperature", t.temperature);
        if (t.has_hum) cJSON_AddNumberToObject(pl, "relative_humidity", t.relative_humidity);
        if (t.has_press) cJSON_AddNumberToObject(pl, "barometric_pressure", t.barometric_pressure);
        type = "telemetry";
        break;
    }
    default:
        break;
    }
    if (!pl) {
        cJSON_Delete(o);
        return;
    }
    char gw[16];
    gateway_id(gw);
    cJSON_AddNumberToObject(o, "channel", ch);
    cJSON_AddNumberToObject(o, "from", h->from);
    cJSON_AddNumberToObject(o, "hop_start", mt_hop_start(h));
    cJSON_AddNumberToObject(o, "hops_away", mt_hop_start(h) >= mt_hop_limit(h) ? mt_hop_start(h) - mt_hop_limit(h) : 0);
    cJSON_AddNumberToObject(o, "id", h->id);
    cJSON_AddItemToObject(o, "payload", pl);
    cJSON_AddNumberToObject(o, "rssi", rssi);
    cJSON_AddStringToObject(o, "sender", gw);
    cJSON_AddNumberToObject(o, "snr", snr);
    cJSON_AddNumberToObject(o, "timestamp", now_unix_s());
    cJSON_AddNumberToObject(o, "to", h->to);
    cJSON_AddStringToObject(o, "type", type);
    char *txt = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (!txt) return;
    char topic[MQTT_TOPIC_MAX];
    snprintf(topic, sizeof(topic), "%s/2/json/%s/%s", s_mqtt_root, s_ch[ch].name, gw);
    mesh_mqtt_publish(topic, txt, strlen(txt));
    cJSON_free(txt);
}

/* Publishes a packet we sent or heard to the broker (Meshtastic MQTT uplink). Caller holds s_lock. */
static void mqtt_uplink(const uint8_t *frame, size_t len, int ch, bool pki, const mt_data_t *d, bool from_us, int16_t rssi,
                        float snr)
{
    if (!s_mod.mqtt_enabled || s_rx_from_mqtt || !mesh_mqtt_connected() || len <= MESH_HEADER_LEN) return;
    mt_header_t h;
    mt_header_read(frame, &h);
    if (h.flags & MT_FLAG_VIA_MQTT) return;
    bool up = pki ? s_mod.ch_uplink != 0 : (ch >= 0 && ch < s_nch && ((s_mod.ch_uplink >> ch) & 1));
    if (!up) return;
    /* Others' packets go up only when their sender allowed it (Data.bitfield OK_TO_MQTT). */
    if (!from_us && (!d || !(d->has_bitfield && (d->bitfield & 1)))) return;
    mt_envelope_t e = {.has_packet = true};
    e.packet.from = h.from;
    e.packet.to = h.to;
    e.packet.id = h.id;
    e.packet.hop_limit = mt_hop_limit(&h);
    e.packet.hop_start = mt_hop_start(&h);
    e.packet.want_ack = mt_want_ack(&h);
    e.packet.next_hop = h.next_hop;
    e.packet.relay_node = h.relay_node;
    e.packet.rx_time = now_unix_s();
    e.packet.rx_snr = snr;
    e.packet.rx_rssi = rssi;
    e.packet.pki_encrypted = pki;
    uint8_t dbuf[MESH_MAX_PAYLOAD];
    if (s_mod.mqtt_encryption || pki || !d) {
        e.packet.channel = h.channel;
        e.packet.payload = frame + MESH_HEADER_LEN;
        e.packet.payload_len = len - MESH_HEADER_LEN;
    } else {
        size_t n = mt_data_encode(d, dbuf, sizeof(dbuf));
        if (!n) return;
        e.packet.decoded = true;
        e.packet.payload = dbuf;
        e.packet.payload_len = n;
        e.packet.channel = (uint32_t)ch;
    }
    edge_strlcpy(e.channel_id, pki ? "PKI" : s_ch[ch].name, sizeof(e.channel_id));
    gateway_id(e.gateway_id);
    uint8_t out[MQTT_RX_MAX];
    size_t n = mt_envelope_encode(&e, out, sizeof(out));
    char topic[MQTT_TOPIC_MAX];
    snprintf(topic, sizeof(topic), "%s/2/e/%s/%s", s_mqtt_root, e.channel_id, e.gateway_id);
    if (n) mesh_mqtt_publish(topic, out, n);
    if (s_mod.mqtt_json && d && !pki && !s_mqtt_public) mqtt_json_uplink(&h, ch, d, rssi, snr);
}

static void mqtt_json_downlink(const uint8_t *buf, size_t n)
{
    cJSON *j = cJSON_ParseWithLength((const char *)buf, n);
    if (!j) return;
    const cJSON *from = cJSON_GetObjectItemCaseSensitive(j, "from");
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(j, "type");
    const cJSON *pl = cJSON_GetObjectItemCaseSensitive(j, "payload");
    const cJSON *to = cJSON_GetObjectItemCaseSensitive(j, "to");
    const cJSON *chn = cJSON_GetObjectItemCaseSensitive(j, "channel");
    /* Meshtastic: a JSON downlink must name this node as "from" and is sent as if typed here. */
    if (cJSON_IsNumber(from) && (uint32_t)from->valuedouble == s_me && cJSON_IsString(type) &&
        strcmp(type->valuestring, "sendtext") == 0 && cJSON_IsString(pl) && pl->valuestring[0]) {
        int ch = cJSON_IsNumber(chn) ? chn->valueint : 0;
        uint32_t dest = cJSON_IsNumber(to) ? (uint32_t)to->valuedouble : MT_BROADCAST;
        size_t len = strlen(pl->valuestring);
        if (ch >= 0 && ch < s_nch && ((s_mod.ch_downlink >> ch) & 1) && len <= MAX_TEXT) {
            mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)pl->valuestring, .payload_len = len};
            send_data(dest, ch, &d, false, false, s_cfg.hop_limit, NULL, NULL);
        }
    }
    cJSON_Delete(j);
}

/* Drains broker messages into the mesh as if heard on air (marked via MQTT). Caller holds s_lock. */
static void mqtt_downlink(void)
{
    char topic[MQTT_TOPIC_MAX];
    size_t n;
    char me[16];
    gateway_id(me);
    for (int k = 0; k < 4 && mesh_mqtt_take(topic, sizeof(topic), s_dl_buf, sizeof(s_dl_buf), &n); k++) {
        if (strstr(topic, "/2/json/")) {
            if (s_mod.mqtt_json) mqtt_json_downlink(s_dl_buf, n);
            continue;
        }
        mt_envelope_t e;
        if (!mt_envelope_decode(s_dl_buf, n, &e)) continue;
        if (strcmp(e.gateway_id, me) == 0 || e.packet.from == s_me || !e.packet.payload) continue;
        bool pki_topic = strcmp(e.channel_id, "PKI") == 0;
        int ch = -1;
        if (pki_topic) {
            if (e.packet.to != s_me) continue;
        } else {
            for (int i = 0; i < s_nch && ch < 0; i++) {
                if (strcmp(s_ch[i].name, e.channel_id) == 0 && ((s_mod.ch_downlink >> i) & 1)) ch = i;
            }
            if (ch < 0) continue;
        }
        size_t plen = e.packet.payload_len;
        if (!plen || plen + MESH_HEADER_LEN > MESH_MAX_PACKET) continue;
        uint8_t hop = (uint8_t)(e.packet.hop_limit > 7 ? 7 : e.packet.hop_limit);
        if (s_mqtt_public && hop > 1) hop = 1; /* public broker: hand it to direct neighbours only */
        uint8_t hs = (uint8_t)(e.packet.hop_start > 7 ? 7 : e.packet.hop_start);
        if (hs < hop) hs = hop;
        mt_header_t h = {.to = e.packet.to, .from = e.packet.from, .id = e.packet.id,
                         .flags = (uint8_t)(mt_make_flags(hop, hs, e.packet.want_ack) | MT_FLAG_VIA_MQTT),
                         .channel = pki_topic ? 0 : s_ch[ch].hash};
        uint8_t frame[MESH_MAX_PACKET];
        mt_header_write(&h, frame);
        memcpy(frame + MESH_HEADER_LEN, e.packet.payload, plen);
        if (e.packet.decoded) {
            if (pki_topic) continue;
            channel_crypt(&s_ch[ch], h.from, h.id, frame + MESH_HEADER_LEN, plen);
        } else if (!pki_topic && e.packet.channel != s_ch[ch].hash) {
            continue;
        }
        s_rx_from_mqtt = true;
        process_rx(frame, MESH_HEADER_LEN + plen, 0, 0);
        s_rx_from_mqtt = false;
    }
}

static void mqtt_map_report(void)
{
    mt_mapreport_t m = {.long_name = s_cfg.long_name, .short_name = s_cfg.short_name, .firmware_version = FW_COMPAT,
                        .role = s_cfg.mx.role, .hw_model = MT_HW_HELTEC_V3, .region = s_cfg.region,
                        .modem_preset = s_cfg.preset, .online_nodes = (uint32_t)node_count()};
    const edge_mesh_channel_t *p = &s_cfg.channels[0];
    m.has_default_channel = p->psk_len == 1 && p->psk[0] == 1 && (!p->name[0] || strcmp(s_ch[0].name, p->name) == 0);
    int32_t la, lo, al;
    if (s_mod.mqtt_map_location && own_position(&la, &lo, &al)) {
        uint8_t bits = s_mod.mqtt_map_precision ? s_mod.mqtt_map_precision : 14;
        m.has_position = true;
        m.lat_i = mt_position_truncate(la, bits);
        m.lon_i = mt_position_truncate(lo, bits);
        m.altitude = al;
        m.position_precision = bits;
        m.opted_location = true;
    }
    uint8_t mr[200];
    size_t ml = mt_mapreport_encode(&m, mr, sizeof(mr));
    if (!ml) return;
    uint8_t db[240];
    mt_data_t d = {.portnum = MT_PORT_MAP_REPORT, .payload = mr, .payload_len = ml};
    size_t dl = mt_data_encode(&d, db, sizeof(db));
    mt_envelope_t e = {.has_packet = true};
    e.packet = (mt_packet_t){.from = s_me, .to = MT_BROADCAST, .id = new_packet_id(), .decoded = true, .payload = db,
                             .payload_len = dl, .rx_time = now_unix_s()};
    edge_strlcpy(e.channel_id, s_ch[0].name, sizeof(e.channel_id));
    gateway_id(e.gateway_id);
    uint8_t out[MQTT_RX_MAX];
    size_t n = mt_envelope_encode(&e, out, sizeof(out));
    char topic[MQTT_TOPIC_MAX];
    snprintf(topic, sizeof(topic), "%s/2/map/", s_mqtt_root);
    if (n) mesh_mqtt_publish(topic, out, n);
}

static void mqtt_manage(void)
{
    int64_t t = now_ms();
    if (t < s_mqtt_check && !s_mqtt_restart) return;
    s_mqtt_check = t + 5000;
    edge_wifi_info_t wi;
    bool want = s_mod.mqtt_enabled && s_region && wifi_up(&wi);
    if (mesh_mqtt_running() && (!want || s_mqtt_restart)) mesh_mqtt_stop();
    s_mqtt_restart = false;
    if (want && !mesh_mqtt_running()) {
        mesh_mqtt_cfg_t mc = {.tls = s_mod.mqtt_tls};
        edge_strlcpy(mc.host, s_mod.mqtt_address, sizeof(mc.host));
        char *colon = strchr(mc.host, ':');
        mc.port = s_mod.mqtt_tls ? 8883 : 1883;
        if (colon) {
            *colon = 0;
            int pt = atoi(colon + 1);
            if (pt > 0 && pt < 65536) mc.port = (uint16_t)pt;
        }
        edge_strlcpy(mc.username, s_mod.mqtt_username, sizeof(mc.username));
        edge_strlcpy(mc.password, s_mod.mqtt_password, sizeof(mc.password));
        gateway_id(mc.client_id);
        bool any_down = false;
        for (int i = 0; i < s_nch && mc.n_subs < MQTT_SUBS_MAX - 2; i++) {
            if (!((s_mod.ch_downlink >> i) & 1)) continue;
            snprintf(mc.subs[mc.n_subs++], MQTT_TOPIC_MAX, "%s/2/e/%s/+", s_mqtt_root, s_ch[i].name);
            any_down = true;
        }
        if (any_down) snprintf(mc.subs[mc.n_subs++], MQTT_TOPIC_MAX, "%s/2/e/PKI/+", s_mqtt_root);
        if (any_down && s_mod.mqtt_json) snprintf(mc.subs[mc.n_subs++], MQTT_TOPIC_MAX, "%s/2/json/mqtt/#", s_mqtt_root);
        mesh_mqtt_start(&mc);
        s_map_next = t + 60000;
    }
    if (mesh_mqtt_connected()) {
        mqtt_downlink();
        if (s_mod.mqtt_map && t >= s_map_next) {
            mqtt_map_report();
            s_map_next = t + (int64_t)s_mod.mqtt_map_interval_s * 1000;
        }
    }
}

/* ---- Store & forward server -------------------------------------------------------------- */

static void sfs_store(uint32_t from, uint32_t to, uint32_t id, int ch, const mt_data_t *d, const uint8_t *text, size_t len)
{
    if (!s_sfs_on || ch < 0 || !len) return;
    sfdb_rec_t r = {.time = now_unix_s(), .from = from, .to = to, .id = id, .reply_id = d ? d->reply_id : 0,
                    .emoji = d ? d->emoji : 0, .channel = (uint8_t)ch, .len = (uint8_t)(len > SFDB_TEXT_MAX ? SFDB_TEXT_MAX : len)};
    memcpy(r.text, text, r.len);
    sfdb_add(&r);
}

static void sfs_reply_rr(uint32_t to, int ch, uint32_t rr, uint32_t req)
{
    uint8_t b[8];
    size_t l = mt_sf_encode_rr(rr, b, sizeof(b));
    send_port(to, ch, MT_PORT_STORE_FWD, b, l, false, req, s_cfg.hop_limit);
}

static void sfs_history(uint32_t client, int ch, uint32_t window_min, uint32_t req)
{
    if (s_sfs.num && s_sfs.num != client && s_sfs.idx < s_sfs.n) {
        sfs_reply_rr(client, ch, MT_SF_ROUTER_BUSY, req);
        return;
    }
    s_sfs_hist_requests++;
    uint32_t win = s_mod.sf_return_window_min;
    if (window_min && window_min < win) win = window_min;
    uint32_t now = now_unix_s();
    uint32_t since = now > win * 60 ? now - win * 60 : 0;
    int slot = -1;
    for (int i = 0; i < 8 && slot < 0; i++) {
        if (s_sfs_last[i].num == client) slot = i;
    }
    uint32_t last = slot >= 0 ? s_sfs_last[slot].last_s : 0;
    if (last > since) since = last;
    int n = sfdb_select(since, client, (uint8_t)ch, s_sfs.seqs, s_mod.sf_return_max);
    uint8_t b[24];
    size_t l = mt_sf_encode_history(MT_SF_ROUTER_HISTORY, (uint32_t)n, win * 60 * 1000, last, b, sizeof(b));
    send_port(client, ch, MT_PORT_STORE_FWD, b, l, false, req, s_cfg.hop_limit);
    s_sfs.num = client;
    s_sfs.ch = ch;
    s_sfs.n = n;
    s_sfs.idx = 0;
    s_sfs.next_at = now_ms() + 4000;
    if (now) {
        if (slot < 0) slot = (int)(client % 8);
        s_sfs_last[slot].num = client;
        s_sfs_last[slot].last_s = now;
    }
}

static void sfs_service(int64_t t, bool polite)
{
    if (!s_sfs_on) return;
    if (s_sfs.num && t >= s_sfs.next_at) {
        if (s_sfs.idx >= s_sfs.n) {
            s_sfs.num = 0;
        } else if (polite) {
            sfdb_rec_t r;
            if (sfdb_read(s_sfs.seqs[s_sfs.idx], &r)) {
                uint8_t sfb[MESH_MAX_PAYLOAD];
                size_t sl = mt_sf_encode_text(r.to == MT_BROADCAST ? MT_SF_ROUTER_TEXT_BROADCAST : MT_SF_ROUTER_TEXT_DIRECT,
                                              r.text, r.len, sfb, sizeof(sfb));
                mt_data_t d = {.portnum = MT_PORT_STORE_FWD, .payload = sfb, .payload_len = sl, .reply_id = r.reply_id,
                               .emoji = r.emoji};
                if (sl) send_data_as(r.from, r.id, s_sfs.num, s_sfs.ch, &d);
            }
            s_sfs.idx++;
            s_sfs.next_at = t + 5000;
        }
    }
    if (s_mod.sf_heartbeat && t >= s_sfs_hb_next) {
        if (polite) {
            uint8_t b[16];
            size_t l = mt_sf_encode_heartbeat(900, 0, b, sizeof(b));
            send_port(MT_BROADCAST, 0, MT_PORT_STORE_FWD, b, l, false, 0, s_cfg.hop_limit);
        }
        s_sfs_hb_next = t + 900 * 1000;
    }
}

static void sfs_handle(const mt_header_t *h, int ch, const mt_sf_t *sf)
{
    s_sfs_requests++;
    switch (sf->rr) {
    case MT_SF_CLIENT_HISTORY:
        sfs_history(h->from, ch, sf->has_history ? sf->window : 0, h->id);
        break;
    case MT_SF_CLIENT_PING:
        sfs_reply_rr(h->from, ch, MT_SF_ROUTER_PONG, h->id);
        break;
    case MT_SF_CLIENT_STATS: {
        mt_sf_stats_t st = {.messages_total = sfdb_total(), .messages_saved = sfdb_count(), .messages_max = sfdb_capacity(),
                            .up_time = (uint32_t)(now_ms() / 1000), .requests = s_sfs_requests,
                            .requests_history = s_sfs_hist_requests, .heartbeat = s_mod.sf_heartbeat,
                            .return_max = s_mod.sf_return_max, .return_window = s_mod.sf_return_window_min};
        uint8_t b[64];
        size_t l = mt_sf_encode_stats(&st, b, sizeof(b));
        send_port(h->from, ch, MT_PORT_STORE_FWD, b, l, false, h->id, s_cfg.hop_limit);
        break;
    }
    case MT_SF_CLIENT_ABORT:
        if (s_sfs.num == h->from) s_sfs.num = 0;
        break;
    default:
        break;
    }
}

/* ---- ATAK ------------------------------------------------------------------------------ */

#define MAX_TAK 16

typedef struct {
    bool used;
    uint32_t node;           /* mesh node that sent it */
    char uid[48];
    char callsign[32];
    uint8_t team, role, battery;
    bool v2, compressed, has_pos;
    int32_t lat_i, lon_i, alt;
    uint16_t speed, course;
    int64_t at;
    uint32_t at_s;
} tak_t;

static tak_t *s_tak;
static uint32_t s_tak_rx;

static const char *TAK_TEAMS[] = {"", "White", "Yellow", "Orange", "Magenta", "Red", "Maroon", "Purple",
                                  "Dark Blue", "Blue", "Cyan", "Teal", "Green", "Dark Green", "Brown"};
static const char *TAK_ROLES[] = {"", "Team Member", "Team Lead", "HQ", "Sniper", "Medic", "Forward Observer", "RTO", "K9"};

static void tak_fill_self(mt_tak_t *t)
{
    memset(t, 0, sizeof(*t));
    t->v2 = true;
    t->team = s_mod.tak_team ? s_mod.tak_team : 10;  /* Cyan, as Meshtastic */
    t->role = s_mod.tak_role ? s_mod.tak_role : 1;   /* Team Member */
    edge_strlcpy(t->callsign, s_cfg.long_name, sizeof(t->callsign));
    edge_strlcpy(t->device_callsign, s_cfg.long_name, sizeof(t->device_callsign));
    snprintf(t->uid, sizeof(t->uid), "!%08lx", (unsigned long)s_me);
    edge_battery_t b;
    edge_battery_get(&b);
    t->battery = b.present ? (uint32_t)b.percent : 0;
}

/* Sends a TAKPacketV2 (port 78, uncompressed: flags 0xFF) on channel ch. */
static esp_err_t tak_send(const mt_tak_t *t, int ch)
{
    uint8_t b[MESH_MAX_PAYLOAD - 24];
    b[0] = MT_TAK_RAW_FLAGS;
    size_t n = mt_takv2_encode(t, b + 1, sizeof(b) - 1);
    if (!n) return ESP_ERR_INVALID_SIZE;
    return send_port(MT_BROADCAST, ch, MT_PORT_ATAK_V2, b, n + 1, false, 0, s_cfg.hop_limit);
}

/* Our position as a TAK position report (TAK_TRACKER role), at the channel's precision. */
static bool tak_send_pli(int ch)
{
    int32_t la, lo, al;
    uint8_t bits = s_ch[ch].precision;
    if (!own_position(&la, &lo, &al) || !bits) return false;
    mt_tak_t t;
    tak_fill_self(&t);
    t.cot_type = MT_COT_A_F_G_U_C;
    bool manual = s_cfg.mx.pos_mode == EDGE_POS_FIXED;
    t.how = manual ? MT_COT_HOW_H_E : MT_COT_HOW_M_G;
    t.geo_src = t.alt_src = manual ? MT_GEO_SRC_USER : MT_GEO_SRC_GPS;
    t.has_pos = true;
    t.lat_i = mt_position_truncate(la, bits);
    t.lon_i = mt_position_truncate(lo, bits);
    t.alt = al;
    return tak_send(&t, ch) == ESP_OK;
}

static void tak_receive(const mt_header_t *h, int ch, bool pki, const mt_data_t *d, int16_t rssi, float snr)
{
    mt_tak_t t;
    bool ok = d->portnum == MT_PORT_ATAK_V2 ? mt_tak_port78_decode(d->payload, d->payload_len, &t)
                                            : mt_takv1_decode(d->payload, d->payload_len, &t);
    if (!ok || !s_tak) return;
    s_tak_rx++;
    /* One row per sender: its uid when readable, else its mesh node. */
    tak_t *slot = NULL;
    for (int i = 0; i < MAX_TAK && !slot; i++) {
        tak_t *x = &s_tak[i];
        if (x->used && (t.uid[0] ? strcmp(x->uid, t.uid) == 0 : (x->node == h->from && !x->uid[0]))) slot = x;
    }
    for (int i = 0; i < MAX_TAK && !slot; i++) {
        if (!s_tak[i].used) slot = &s_tak[i];
    }
    if (!slot) {
        slot = &s_tak[0];
        for (int i = 1; i < MAX_TAK; i++) {
            if (s_tak[i].at < slot->at) slot = &s_tak[i];
        }
    }
    if (!slot->used || slot->node != h->from) memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->node = h->from;
    slot->v2 = t.v2;
    slot->compressed = t.compressed;
    if (t.uid[0]) edge_strlcpy(slot->uid, t.uid, sizeof(slot->uid));
    if (t.callsign[0]) edge_strlcpy(slot->callsign, t.callsign, sizeof(slot->callsign));
    if (t.team) slot->team = (uint8_t)(t.team < 15 ? t.team : 0);
    if (t.role) slot->role = (uint8_t)(t.role < 9 ? t.role : 0);
    if (t.battery) slot->battery = (uint8_t)(t.battery > 100 ? 100 : t.battery);
    if (t.has_pos) {
        slot->has_pos = true;
        slot->lat_i = t.lat_i;
        slot->lon_i = t.lon_i;
        slot->alt = t.alt;
        slot->speed = (uint16_t)t.speed;
        slot->course = (uint16_t)t.course;
    }
    slot->at = now_ms();
    slot->at_s = now_unix_s();
    if (t.has_chat && t.chat[0]) {
        char text[MAX_TEXT + 34];
        int l = snprintf(text, sizeof(text), "%s: %s", slot->callsign[0] ? slot->callsign : "?", t.chat);
        if (l > 0) store_text(h, ch, pki, d, (const uint8_t *)text, (size_t)(l < (int)sizeof(text) ? l : (int)sizeof(text) - 1),
                              rssi, snr, "[TAK] ");
    }
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_tak");
    char id[12];
    node_id_str(h->from, id);
    cJSON_AddStringToObject(ev, "from", id);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

/* ---- Remote admin ------------------------------------------------------------------------ */

static void passkey_refresh(void)
{
    int64_t t = now_ms();
    if (s_passkey_at < 0 || t - s_passkey_at > 150000) {
        esp_fill_random(s_passkey, sizeof(s_passkey));
        s_passkey_at = t;
    }
}

static bool passkey_ok(const uint8_t k[8])
{
    return s_passkey_at >= 0 && now_ms() - s_passkey_at < 300000 && memcmp(k, s_passkey, 8) == 0;
}

static bool admin_key_listed(const node_t *n)
{
    if (!n || !(n->flags & NF_PKC)) return false;
    for (int i = 0; i < s_mod.n_admin_keys; i++) {
        if (memcmp(s_mod.admin_keys[i], n->pubkey, 32) == 0) return true;
    }
    return false;
}

static bool chan_named(int ch, const char *name)
{
    return ch >= 0 && ch < s_nch && strcasecmp(s_ch[ch].name, name) == 0;
}

static void publish_remote(const char *ev_name, uint32_t from, uint32_t what, const char *outcome)
{
    cJSON *ev = cJSON_CreateObject();
    char id[12];
    node_id_str(from, id);
    cJSON_AddStringToObject(ev, "ev", ev_name);
    cJSON_AddStringToObject(ev, "from", id);
    cJSON_AddNumberToObject(ev, "request", what);
    cJSON_AddStringToObject(ev, "outcome", outcome);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void ignore_list_edit(edge_config_t *c, uint32_t num, bool add)
{
    int k = 0;
    for (int i = 0; i < c->mx.n_ignore; i++) {
        if (c->mx.ignore[i] != num) c->mx.ignore[k++] = c->mx.ignore[i];
    }
    c->mx.n_ignore = (uint8_t)k;
    if (add && c->mx.n_ignore < EDGE_IGNORE_MAX) c->mx.ignore[c->mx.n_ignore++] = num;
}

static void handle_admin(const mt_header_t *h, int ch, bool pki, const mt_data_t *d)
{
    int reply_ch = ch < 0 ? 0 : ch;
    s_admin_seen++;
    node_t *n = node_get(h->from, false);
    bool key_ok = pki && admin_key_listed(n);
    bool sens_ok = key_ok || (chan_named(ch, "admin") && s_mod.admin_channel);
    bool allowed = s_mod.admin_enabled && (sens_ok || s_mod.admin_mode == EDGE_ADMIN_OPEN);
    adm_class_t cls;
    if (!mesh_admin_classify(d->payload, d->payload_len, &cls) || !cls.variant) {
        send_routing(h->from, h->id, reply_ch, 32 /* BAD_REQUEST */);
        return;
    }
    uint32_t err = 0;
    if (!allowed) err = pki ? 37 /* ADMIN_PUBLIC_KEY_UNAUTHORIZED */ : 33 /* NOT_AUTHORIZED */;
    else if (cls.sensitive && !sens_ok) err = 33;
    else if (!cls.is_get && !(cls.has_passkey && passkey_ok(cls.passkey))) err = 36; /* ADMIN_BAD_SESSION_KEY */
    if (err) {
        s_admin_refused++;
        send_routing(h->from, h->id, reply_ch, err);
        publish_remote("mesh_admin", h->from, cls.variant, err == 36 ? "refused: no session key" : "refused");
        return;
    }
    passkey_refresh();
    struct {
        edge_config_t cfg;
        edge_mesh_mod_t mod;
        adm_result_t res;
        adm_ctx_t x;
        edge_wifi_info_t wi;
    } *w = calloc(1, sizeof(*w));
    if (!w) return;
    w->cfg = s_cfg;
    w->mod = s_mod;
    adm_ctx_t *x = &w->x;
    x->my_node = s_me;
    x->public_key = s_have_keys ? s_pub : NULL;
    x->fw_version = FW_COMPAT;
    x->passkey = s_passkey;
    x->sensitive_ok = sens_ok;
    x->sf_capacity = sfdb_capacity();
    x->wifi_connected = wifi_up(&w->wi);
    if (x->wifi_connected) {
        unsigned a, b, c2, d2;
        if (sscanf(w->wi.ip, "%u.%u.%u.%u", &a, &b, &c2, &d2) == 4) x->ip = a | b << 8 | c2 << 16 | d2 << 24;
        edge_strlcpy(x->ssid, w->wi.ssid, sizeof(x->ssid));
        x->wifi_rssi = w->wi.rssi;
    }
    x->mqtt_connected = mesh_mqtt_connected();
    if (!mesh_admin_apply(d->payload, d->payload_len, x, &w->cfg, &w->mod, &w->res)) {
        send_routing(h->from, h->id, reply_ch, 32);
        free(w);
        return;
    }
    adm_result_t *r = &w->res;
    if (r->reply_len) send_port(h->from, reply_ch, MT_PORT_ADMIN, r->reply, r->reply_len, false, h->id, s_cfg.hop_limit);
    if (r->ign_set || r->ign_clear) {
        if (r->ign_set) ignore_list_edit(&w->cfg, r->ign_set, true);
        if (r->ign_clear) ignore_list_edit(&w->cfg, r->ign_clear, false);
        r->cfg_changed = true;
    }
    if (r->factory_config) {
        edge_config_t keep = w->cfg;
        edge_config_defaults(&w->cfg);
        w->cfg.mode = keep.mode;
        w->cfg.link_mode = keep.link_mode;
        w->cfg.node_num = keep.node_num;
        edge_strlcpy(w->cfg.long_name, keep.long_name, sizeof(w->cfg.long_name));
        edge_strlcpy(w->cfg.short_name, keep.short_name, sizeof(w->cfg.short_name));
        edge_mod_defaults(&w->mod);
        r->cfg_changed = r->mod_changed = true;
    }
    if (r->cfg_changed) {
        if (w->cfg.link_mode != s_cfg.link_mode) s_wifi_reconf = true;
        edge_config_set(&w->cfg);
    }
    if (r->mod_changed) edge_mod_set(&w->mod);
    int64_t t = now_ms();
    if (r->cfg_changed || r->mod_changed) s_reload_at = t + 3000; /* after the reply has gone out */
    if (r->owner_changed) s_nodeinfo_at = t + 8000;
    if (r->wifi_add) edge_wifi_net_add(r->wifi_ssid, r->wifi_psk);
    if (r->set_time && r->time > 1600000000u) edge_time_set_unix_ms((int64_t)r->time * 1000, "mesh admin");
    uint32_t nums[] = {r->fav_set, r->fav_clear, r->ign_set, r->ign_clear};
    for (int i = 0; i < 4; i++) {
        node_t *nn = nums[i] ? node_get(nums[i], false) : NULL;
        if (!nn) continue;
        if (i == 0) nn->flags |= NF_FAV;
        if (i == 1) nn->flags &= (uint8_t)~NF_FAV;
        if (i == 2) nn->flags |= NF_IGNORE;
        if (i == 3) nn->flags &= (uint8_t)~NF_IGNORE;
        s_dirty = true;
    }
    if (r->remove_node) {
        node_t *nn = node_get(r->remove_node, false);
        if (nn) {
            memset(nn, 0, sizeof(*nn));
            s_dirty = true;
        }
    }
    if (r->add_contact) {
        node_t *nn = node_get(r->contact_num, true);
        if (nn) {
            nn->flags |= NF_USER;
            edge_strlcpy(nn->long_name, r->contact_long, sizeof(nn->long_name));
            edge_strlcpy(nn->short_name, r->contact_short, sizeof(nn->short_name));
            nn->hw_model = (uint8_t)r->contact_hw;
            if (r->contact_has_key) {
                memcpy(nn->pubkey, r->contact_key, 32);
                nn->flags |= NF_PKC;
                nn->key_ok = false;
            }
            s_dirty = true;
        }
    }
    if (r->nodedb_reset) {
        memset(s_nodes, 0, MAX_NODES * sizeof(node_t));
        s_dirty = true;
    }
    if (r->factory_device) edge_app_factory_reset_later(3000);
    if (r->reboot_s >= 0) s_reboot_at = t + (int64_t)r->reboot_s * 1000 + 3000;
    publish_remote("mesh_admin", h->from, cls.variant, r->unsupported ? "not supported" : cls.is_get ? "answered" : "applied");
    if (!cls.is_get && !r->unsupported) {
        char det[40];
        snprintf(det, sizeof(det), "remote admin from !%08lx", (unsigned long)h->from);
        edge_record_event(EDGE_EVT_CONFIG, det);
    }
    free(w);
}

/* ---- Remote hardware ------------------------------------------------------------------- */

static bool pin_in_use(int g)
{
    const edge_mesh_ext_t *x = &s_cfg.mx;
    return g == x->serial_rx || g == x->serial_tx || g == x->detect_gpio || g == x->notify_gpio || g == s_mod.audio_ptt ||
           g == s_mod.i2s_ws || g == s_mod.i2s_sd || g == s_mod.i2s_din || g == s_mod.i2s_sck;
}

static uint64_t rhw_allowed(bool write)
{
    uint64_t m = 0;
    for (int i = 0; i < s_mod.n_pins; i++) {
        int g = s_mod.pins[i].gpio;
        if (edge_pin_protected(g) || pin_in_use(g)) continue;
        if (!write || s_mod.pins[i].type == EDGE_RHW_WRITE) m |= 1ULL << g;
    }
    if (s_mod.rhw_undefined_ok) {
        for (int g = 0; g <= 48; g++) {
            if (!edge_pin_protected(g) && !pin_in_use(g) && GPIO_IS_VALID_GPIO(g)) m |= 1ULL << g;
        }
    }
    return m;
}

static uint64_t rhw_read(uint64_t mask)
{
    uint64_t v = 0;
    for (int g = 0; g <= 48; g++) {
        if (!(mask & (1ULL << g))) continue;
        if (!(s_rhw_out & (1ULL << g))) gpio_set_direction(g, GPIO_MODE_INPUT);
        if (gpio_get_level(g)) v |= 1ULL << g;
    }
    return v;
}

static void handle_rhw(const mt_header_t *h, int ch, bool pki, const mt_data_t *d)
{
    if (!s_mod.rhw_enabled || h->to != s_me) return;
    int reply_ch = ch < 0 ? 0 : ch;
    bool allowed = (pki && admin_key_listed(node_get(h->from, false))) || chan_named(ch, "gpio") ||
                   (chan_named(ch, "admin") && s_mod.admin_channel) || s_mod.admin_mode == EDGE_ADMIN_OPEN;
    if (!allowed) {
        send_routing(h->from, h->id, reply_ch, 33);
        publish_remote("mesh_rhw", h->from, 0, "refused");
        return;
    }
    mt_hw_t m;
    if (!mt_hw_decode(d->payload, d->payload_len, &m)) return;
    switch (m.type) {
    case MT_HW_WRITE_GPIOS: {
        uint64_t mask = m.mask & rhw_allowed(true);
        for (int g = 0; g <= 48; g++) {
            if (!(mask & (1ULL << g))) continue;
            gpio_set_direction(g, GPIO_MODE_INPUT_OUTPUT);
            gpio_set_level(g, (m.value >> g) & 1);
            s_rhw_out |= 1ULL << g;
        }
        break;
    }
    case MT_HW_READ_GPIOS: {
        uint64_t mask = m.mask & rhw_allowed(false);
        mt_hw_t rep = {.type = MT_HW_READ_GPIOS_REPLY, .mask = mask, .value = rhw_read(mask)};
        uint8_t b[32];
        size_t l = mt_hw_encode(&rep, b, sizeof(b));
        send_port(h->from, reply_ch, MT_PORT_REMOTE_HW, b, l, false, h->id, s_cfg.hop_limit);
        break;
    }
    case MT_HW_WATCH_GPIOS:
        s_rhw_watch = m.mask & rhw_allowed(false);
        s_rhw_watcher = h->from;
        s_rhw_ch = reply_ch;
        s_rhw_last = rhw_read(s_rhw_watch);
        break;
    default:
        break;
    }
    publish_remote("mesh_rhw", h->from, m.type, "done");
}

static void rhw_poll(int64_t t)
{
    if (!s_rhw_watch || t - s_rhw_sent < 1000) return;
    uint64_t v = rhw_read(s_rhw_watch);
    if (v == s_rhw_last) return;
    s_rhw_last = v;
    s_rhw_sent = t;
    mt_hw_t rep = {.type = MT_HW_GPIOS_CHANGED, .mask = s_rhw_watch, .value = v};
    uint8_t b[32];
    size_t l = mt_hw_encode(&rep, b, sizeof(b));
    send_port(s_rhw_watcher, s_rhw_ch, MT_PORT_REMOTE_HW, b, l, false, 0, s_cfg.hop_limit);
}

/* ---- Receive handlers ------------------------------------------------------------------ */

static void notify_message(const char *text)
{
    uint8_t mode = s_cfg.mx.notify_mode;
    if (mode == 0 || (mode == 2 && !strchr(text, '\a'))) return;
    s_notify_until = now_ms() + 5000;
}

static void serial_out(uint32_t from, const char *text)
{
    if (!s_serial_on) return;
    char line[MAX_TEXT + 48];
    node_t *n = node_get(from, false);
    int l = snprintf(line, sizeof(line), "%s: %s\r\n", n && (n->flags & NF_USER) ? n->short_name : "?", text);
    if (l > 0) uart_write_bytes(UART_NUM_1, line, (size_t)(l < (int)sizeof(line) ? l : (int)sizeof(line) - 1));
}

static void store_text(const mt_header_t *h, int ch, bool pki, const mt_data_t *d, const uint8_t *text, size_t len,
                       int16_t rssi, float snr, const char *prefix)
{
    msg_t *m = msg_add();
    m->outgoing = false;
    m->pki = pki;
    m->id = h->id;
    m->from = d->source ? d->source : h->from;
    m->to = h->to;
    m->channel = (uint8_t)(ch < 0 ? 0 : ch);
    m->rssi = rssi;
    m->snr_x4 = (int8_t)(snr * 4);
    m->reply_id = d->reply_id;
    m->emoji = d->emoji;
    char clean[MAX_TEXT + 34];
    edge_sanitize_text(clean, sizeof(clean), text, len);
    snprintf(m->text, sizeof(m->text), "%s%s", prefix ? prefix : "", clean);

    log_text(m->from, h->to, h->id, m->channel, (uint8_t)(mt_hop_limit(h) | (mt_hop_start(h) << 4)), rssi,
             (int8_t)(snr * 4), false, m->text, strlen(m->text));
    notify_message(m->text);
    serial_out(m->from, m->text);

    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_rx");
    cJSON_AddItemToObject(ev, "msg", msg_json(m));
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void insert_unknown_hops(const mt_header_t *h, uint32_t *route, uint8_t *n_route, int8_t *snr, uint8_t *n_snr)
{
    int taken = mt_hop_start(h) > 0 ? mt_hop_start(h) - mt_hop_limit(h) : 0;
    while (*n_route < taken && *n_route < MT_MAX_ROUTE) route[(*n_route)++] = MT_BROADCAST;
    while (*n_snr < *n_route && *n_snr < MT_MAX_ROUTE) snr[(*n_snr)++] = INT8_MIN;
}

static void route_json(cJSON *ev, const char *key, const uint32_t *r, int n)
{
    char id[12];
    cJSON *a = cJSON_AddArrayToObject(ev, key);
    for (int i = 0; i < n; i++) {
        node_id_str(r[i], id);
        cJSON_AddItemToArray(a, cJSON_CreateString(r[i] == MT_BROADCAST ? "unknown" : id));
    }
}

static void snr_json(cJSON *ev, const char *key, const int8_t *s, int n)
{
    cJSON *a = cJSON_AddArrayToObject(ev, key);
    for (int i = 0; i < n; i++) cJSON_AddItemToArray(a, s[i] == INT8_MIN ? cJSON_CreateNull() : cJSON_CreateNumber(s[i] / 4.0));
}

static void publish_traceroute(uint32_t req_id, uint32_t dest, const mt_route_t *r)
{
    cJSON *ev = cJSON_CreateObject();
    char id[12];
    cJSON_AddStringToObject(ev, "ev", "mesh_traceroute");
    cJSON_AddNumberToObject(ev, "id", req_id);
    node_id_str(dest, id);
    cJSON_AddStringToObject(ev, "to", id);
    route_json(ev, "route", r->route, r->n_route);
    snr_json(ev, "snr_towards", r->snr_towards, r->n_snr_towards);
    route_json(ev, "route_back", r->route_back, r->n_route_back);
    snr_json(ev, "snr_back", r->snr_back, r->n_snr_back);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

/* Returns true and fills relay_plain (re-encoded Data) when the packet must be relayed modified. */
static bool handle_data(const mt_header_t *h, int ch, bool pki, const mt_data_t *d, int16_t rssi, float snr,
                        uint8_t *relay_plain, size_t *relay_len)
{
    node_t *n = node_get(h->from, true);
    bool for_me = h->to == s_me;
    bool for_us = for_me || h->to == MT_BROADCAST;
    int8_t snr4 = (int8_t)(snr * 4);
    int reply_ch = ch < 0 ? 0 : ch;
    switch (d->portnum) {
    case MT_PORT_TEXT:
        if (s_sfs_on && for_me && d->payload_len == 2 && strncasecmp((const char *)d->payload, "SF", 2) == 0) {
            sfs_history(h->from, reply_ch, 0, h->id); /* "SF" typed to a server asks for history */
            break;
        }
        if (s_sfs_on && !pki && ch >= 0) sfs_store(h->from, h->to, h->id, ch, d, d->payload, d->payload_len);
        if (for_us && d->payload_len) store_text(h, ch, pki, d, d->payload, d->payload_len, rssi, snr, NULL);
        break;
    case MT_PORT_ADMIN:
        if (for_me) handle_admin(h, ch, pki, d);
        break;
    case MT_PORT_REMOTE_HW:
        if (for_me) handle_rhw(h, ch, pki, d);
        break;
    case MT_PORT_AUDIO:
        s_audio_frames++; /* codec2 playback needs the speaker hardware (not fitted) */
        break;
    case MT_PORT_ATAK:
    case MT_PORT_ATAK_V2:
        if (for_us && d->payload_len) tak_receive(h, ch, pki, d, rssi, snr);
        break;
    case MT_PORT_RANGE_TEST:
        if (for_us && d->payload_len) store_text(h, ch, pki, d, d->payload, d->payload_len, rssi, snr, "[range test] ");
        break;
    case MT_PORT_DETECTION:
        if (for_us && d->payload_len) store_text(h, ch, pki, d, d->payload, d->payload_len, rssi, snr, "[sensor] ");
        break;
    case MT_PORT_NODEINFO: {
        mt_user_t u;
        if (n && mt_user_decode(d->payload, d->payload_len, &u)) {
            n->flags |= NF_USER;
            edge_strlcpy(n->long_name, u.long_name, sizeof(n->long_name));
            edge_strlcpy(n->short_name, u.short_name, sizeof(n->short_name));
            n->hw_model = (uint8_t)u.hw_model;
            n->role = (uint8_t)u.role;
            if (u.has_public_key && (!(n->flags & NF_PKC) || memcmp(n->pubkey, u.public_key, 32) != 0)) {
                memcpy(n->pubkey, u.public_key, 32);
                n->flags |= NF_PKC;
                n->key_ok = false;
            }
            s_dirty = true;
            publish_node(n);
        }
        if (d->want_response && for_us && n && now_ms() - n->last_nodeinfo_reply > 10 * 60 * 1000) {
            n->last_nodeinfo_reply = now_ms();
            send_nodeinfo(h->from, false);
        }
        break;
    }
    case MT_PORT_POSITION: {
        mt_position_t p;
        if (n && mt_position_decode(d->payload, d->payload_len, &p) && p.has_lat && p.has_lon) {
            n->flags |= NF_POS;
            n->lat_i = p.lat_i;
            n->lon_i = p.lon_i;
            n->alt = p.has_alt ? p.alt : 0;
            n->pos_time = p.time;
            n->pos_bits = (uint8_t)p.precision_bits;
            s_dirty = true;
        }
        if (d->want_response && for_me) send_position(h->from, reply_ch, false, h->id);
        break;
    }
    case MT_PORT_TELEMETRY: {
        mt_telemetry_t t;
        if (n && mt_telemetry_decode(d->payload, d->payload_len, &t)) {
            if (t.has_device) {
                n->flags |= NF_DEV;
                if (t.has_battery) n->battery = (uint8_t)(t.battery_level > 255 ? 255 : t.battery_level);
                if (t.has_voltage) n->voltage_mv = (uint16_t)(t.voltage * 1000);
                if (t.has_ch_util) n->ch_util_x10 = (uint16_t)(t.channel_utilization * 10);
                if (t.has_air_util) n->air_util_x10 = (uint16_t)(t.air_util_tx * 10);
                if (t.has_uptime) n->uptime_s = t.uptime_seconds;
            }
            if (t.has_env) {
                n->flags |= NF_ENV;
                if (t.has_temp) n->temp_x10 = (int16_t)(t.temperature * 10);
                if (t.has_hum) n->hum_x10 = (uint16_t)(t.relative_humidity * 10);
                if (t.has_press) n->press_x10 = (uint16_t)(t.barometric_pressure * 10);
            }
            s_dirty = true;
        }
        if (d->want_response && for_me) send_telemetry(h->from, h->id);
        break;
    }
    case MT_PORT_NEIGHBORINFO: {
        mt_neighborinfo_t ni;
        if (mt_neighborinfo_decode(d->payload, d->payload_len, &ni)) {
            node_t *src = node_get(ni.node_id ? ni.node_id : h->from, true);
            if (src) {
                src->n_nb = 0;
                for (int i = 0; i < ni.n && src->n_nb < MAX_NB; i++) {
                    src->nb[src->n_nb].num = ni.nb[i].node_id;
                    src->nb[src->n_nb].snr_x4 = (int8_t)(ni.nb[i].snr * 4);
                    src->n_nb++;
                }
                s_dirty = true;
            }
        }
        break;
    }
    case MT_PORT_WAYPOINT: {
        mt_waypoint_t w;
        if (s_wps && mt_waypoint_decode(d->payload, d->payload_len, &w)) {
            wp_t *slot = NULL;
            for (int i = 0; i < MAX_WAYPOINTS; i++) {
                if (s_wps[i].used && s_wps[i].w.id == w.id) slot = &s_wps[i];
            }
            for (int i = 0; i < MAX_WAYPOINTS && !slot; i++) {
                if (!s_wps[i].used) slot = &s_wps[i];
            }
            if (!slot) slot = &s_wps[rnd(MAX_WAYPOINTS)];
            slot->used = true;
            slot->from = h->from;
            slot->at_s = now_unix_s();
            slot->w = w;
            s_dirty = true;
            cJSON *ev = cJSON_CreateObject();
            cJSON_AddStringToObject(ev, "ev", "mesh_waypoint");
            cJSON_AddNumberToObject(ev, "id", w.id);
            edge_events_publish(ev);
            cJSON_Delete(ev);
        }
        break;
    }
    case MT_PORT_PAXCOUNTER: {
        uint32_t wf, bl, up;
        if (for_us && mt_paxcount_decode(d->payload, d->payload_len, &wf, &bl, &up)) {
            char t[64];
            int l = snprintf(t, sizeof(t), "Wi-Fi %lu, BLE %lu", (unsigned long)wf, (unsigned long)bl);
            store_text(h, ch, pki, d, (const uint8_t *)t, (size_t)l, rssi, snr, "[paxcounter] ");
        }
        break;
    }
    case MT_PORT_STORE_FWD: {
        mt_sf_t sf;
        if (!mt_sf_decode(d->payload, d->payload_len, &sf)) break;
        if (s_sfs_on && for_me && sf.rr >= MT_SF_CLIENT_ERROR) {
            sfs_handle(h, reply_ch, &sf);
            break;
        }
        if (sf.rr == MT_SF_ROUTER_HEARTBEAT) {
            int slot = -1;
            for (int i = 0; i < MAX_SF_ROUTERS && slot < 0; i++) {
                if (s_sf[i].num == h->from) slot = i;
            }
            for (int i = 0; i < MAX_SF_ROUTERS && slot < 0; i++) {
                if (!s_sf[i].num) slot = i;
            }
            if (slot < 0) slot = 0;
            s_sf[slot].num = h->from;
            s_sf[slot].at = now_ms();
        } else if ((sf.rr == MT_SF_ROUTER_TEXT_DIRECT || sf.rr == MT_SF_ROUTER_TEXT_BROADCAST) && sf.text_len && for_us) {
            store_text(h, ch, pki, d, sf.text, sf.text_len, rssi, snr, "[stored] ");
        } else if (sf.rr == MT_SF_ROUTER_HISTORY && for_me) {
            char t[64];
            int l = snprintf(t, sizeof(t), "%lu messages on their way", (unsigned long)sf.history_messages);
            store_text(h, ch, pki, d, (const uint8_t *)t, (size_t)l, rssi, snr, "[store & forward] ");
        } else if (sf.text_len && for_me) {
            store_text(h, ch, pki, d, sf.text, sf.text_len, rssi, snr, "[store & forward] ");
        }
        break;
    }
    case MT_PORT_ROUTING: {
        if (!for_me || !d->request_id) break;
        mt_routing_t r;
        if (!mt_routing_decode(d->payload, d->payload_len, &r)) break;
        for (int i = 0; i < MAX_PENDING; i++) {
            pending_t *p = &s_pending[i];
            if (!p->used || p->id != d->request_id) continue;
            p->used = false;
            node_t *dest = node_get(p->to, false);
            if (!r.has_error || r.error_reason == 0) {
                /* Next-hop learning: whoever handed us the ACK reaches the destination. */
                if (dest && h->from == p->to) {
                    dest->next_hop = h->relay_node;
                    s_dirty = true;
                }
                publish_tx_status(d->request_id, "delivered", -1);
            } else {
                if (dest) dest->next_hop = 0;
                publish_tx_status(d->request_id, "failed", (int)r.error_reason);
            }
        }
        break;
    }
    case MT_PORT_TRACEROUTE: {
        mt_route_t r;
        if (!mt_route_decode(d->payload, d->payload_len, &r)) break;
        if (for_me && d->want_response) {
            /* We are the destination: add our SNR and send it back. */
            insert_unknown_hops(h, r.route, &r.n_route, r.snr_towards, &r.n_snr_towards);
            if (r.n_snr_towards <= MT_MAX_ROUTE) r.snr_towards[r.n_snr_towards++] = snr4;
            uint8_t rb[160];
            size_t rl = mt_route_encode(&r, rb, sizeof(rb));
            if (rl) send_port(h->from, reply_ch, MT_PORT_TRACEROUTE, rb, rl, false, h->id, s_cfg.hop_limit);
        } else if (for_me && d->request_id) {
            /* The answer to our own traceroute. */
            insert_unknown_hops(h, r.route_back, &r.n_route_back, r.snr_back, &r.n_snr_back);
            if (r.n_snr_back <= MT_MAX_ROUTE) r.snr_back[r.n_snr_back++] = snr4;
            publish_traceroute(d->request_id, h->from, &r);
        } else if (!for_us && relay_plain && !pki) {
            /* Relaying: append ourselves so the route is complete. */
            if (d->want_response) {
                insert_unknown_hops(h, r.route, &r.n_route, r.snr_towards, &r.n_snr_towards);
                if (r.n_route < MT_MAX_ROUTE) {
                    r.route[r.n_route++] = s_me;
                    if (r.n_snr_towards <= MT_MAX_ROUTE) r.snr_towards[r.n_snr_towards++] = snr4;
                }
            } else {
                insert_unknown_hops(h, r.route_back, &r.n_route_back, r.snr_back, &r.n_snr_back);
                if (r.n_route_back < MT_MAX_ROUTE) {
                    r.route_back[r.n_route_back++] = s_me;
                    if (r.n_snr_back <= MT_MAX_ROUTE) r.snr_back[r.n_snr_back++] = snr4;
                }
            }
            uint8_t rb[160];
            size_t rl = mt_route_encode(&r, rb, sizeof(rb));
            mt_data_t nd = *d;
            nd.payload = rb;
            nd.payload_len = rl;
            size_t l = rl ? mt_data_encode(&nd, relay_plain, MESH_MAX_PAYLOAD) : 0;
            if (l) {
                *relay_len = l;
                return true;
            }
        }
        break;
    }
    default:
        break;
    }
    return false;
}

static bool core_port(uint32_t p)
{
    return p == MT_PORT_TEXT || p == MT_PORT_POSITION || p == MT_PORT_NODEINFO || p == MT_PORT_ROUTING ||
           p == MT_PORT_TELEMETRY;
}

static void process_rx(uint8_t *buf, size_t len, int16_t rssi, float snr)
{
    if (len < MESH_HEADER_LEN) return;
    mt_header_t h;
    mt_header_read(buf, &h);
    uint8_t *payload = buf + MESH_HEADER_LEN;
    size_t plen = len - MESH_HEADER_LEN;
    if (!s_rx_from_mqtt) air_add(mesh_airtime_ms(s_preset, len), false);

    /* ignore_mqtt covers packets heard over LoRa that crossed MQTT elsewhere, not our own downlink */
    if (is_ignored(h.from) || ((h.flags & MT_FLAG_VIA_MQTT) && s_cfg.mx.ignore_mqtt && !s_rx_from_mqtt)) {
        s_st.ignored++;
        return;
    }
    if (h.from == s_me) {
        for (int i = 0; i < MAX_PENDING; i++) {
            if (s_pending[i].used && s_pending[i].broadcast && s_pending[i].id == h.id) {
                s_pending[i].used = false;
                publish_tx_status(h.id, "relayed", -1);
            }
        }
        return;
    }
    if (seen_find(h.from, h.id)) {
        s_st.rx_dupe++;
        if (txq_cancel_relay(h.from, h.id)) s_st.relay_cancelled++;
        return;
    }
    seen_t *seen = seen_add(h.from, h.id, h.to);
    s_st.rx_ok++;

    node_t *n = node_get(h.from, true);
    if (n && s_rx_from_mqtt) {
        n->last_heard_up = now_ms();
        n->last_heard_s = now_unix_s();
    } else if (n) {
        n->last_heard_up = now_ms();
        n->last_heard_s = now_unix_s();
        n->rssi = rssi;
        n->snr_x4 = (int8_t)(snr * 4);
        if (mt_hop_start(&h) >= mt_hop_limit(&h) && mt_hop_start(&h) > 0) {
            n->hops_away = (int8_t)(mt_hop_start(&h) - mt_hop_limit(&h));
        }
    }

    bool decoded = false, pki = false;
    int ch_idx = -1;
    mt_data_t d;
    uint8_t tmp[MESH_MAX_PAYLOAD];
    if (h.channel == 0 && h.to == s_me && n && plen > MESH_PKI_OVERHEAD && node_key(n)) {
        if (mesh_pki_decrypt(n->shared, h.from, h.id, payload, plen, tmp) &&
            mt_data_decode(tmp, plen - MESH_PKI_OVERHEAD, &d)) {
            decoded = pki = true;
            s_st.rx_pki++;
        }
    }
    for (int i = 0; i < s_nch && !decoded; i++) {
        if (s_ch[i].hash != h.channel) continue;
        memcpy(tmp, payload, plen);
        channel_crypt(&s_ch[i], h.from, h.id, tmp, plen);
        if (mt_data_decode(tmp, plen, &d)) {
            decoded = true;
            ch_idx = i;
        }
    }

    uint8_t relay_plain[MESH_MAX_PAYLOAD];
    size_t relay_len = 0;
    bool modified = false;
    if (decoded || ch_idx >= 0) mqtt_uplink(buf, len, ch_idx, pki, decoded ? &d : NULL, false, rssi, snr);
    if (decoded) {
        modified = handle_data(&h, ch_idx, pki, &d, rssi, snr, ch_idx >= 0 ? relay_plain : NULL, &relay_len);
        if (h.to == s_me && mt_want_ack(&h) && s_cfg.mx.tx_enabled) {
            if (pki) {
                uint8_t rp[8];
                size_t rl = mt_routing_encode_error(0, rp, sizeof(rp));
                mt_data_t ack = {.portnum = MT_PORT_ROUTING, .payload = rp, .payload_len = rl, .request_id = h.id};
                send_data(h.from, 0, &ack, false, false, s_cfg.hop_limit, NULL, NULL); /* PKI: key is known */
            } else {
                send_routing(h.from, h.id, ch_idx, 0);
            }
        }
    } else {
        s_st.rx_undecoded++;
    }

    /* ---- relay decision ---- */
    uint8_t role = s_cfg.mx.role;
    uint8_t mode = s_cfg.rebroadcast;
    bool relay = h.to != s_me && mt_hop_limit(&h) > 0 && s_cfg.mx.tx_enabled && role != EDGE_ROLE_CLIENT_MUTE &&
                 mode != EDGE_REBROADCAST_NONE;
    if (relay && mode == EDGE_REBROADCAST_LOCAL_ONLY) relay = decoded;
    if (relay && mode == EDGE_REBROADCAST_KNOWN_ONLY) relay = n && (n->flags & NF_USER);
    if (relay && mode == EDGE_REBROADCAST_CORE_PORTNUMS_ONLY) relay = decoded && core_port(d.portnum);
    /* Next-hop routing: when another node is the designated relay, stay quiet. */
    uint8_t my_byte = (uint8_t)(s_me & 0xff);
    if (relay && h.next_hop && h.next_hop != my_byte) relay = false;
    if (!relay) return;

    tx_item_t *t = txq_alloc();
    if (!t) return;
    memcpy(t->buf, buf, len);
    t->len = (uint8_t)len;
    if (modified && ch_idx >= 0 && relay_len + MESH_HEADER_LEN <= MESH_MAX_PACKET) {
        memcpy(t->buf + MESH_HEADER_LEN, relay_plain, relay_len);
        channel_crypt(&s_ch[ch_idx], h.from, h.id, t->buf + MESH_HEADER_LEN, relay_len);
        t->len = (uint8_t)(MESH_HEADER_LEN + relay_len);
    }
    t->buf[12] = (uint8_t)((h.flags & ~MT_FLAG_HOP_LIMIT_MASK) | ((mt_hop_limit(&h) - 1) & MT_FLAG_HOP_LIMIT_MASK));
    node_t *dest = node_get(h.to, false);
    t->buf[14] = (h.next_hop == my_byte && dest) ? dest->next_hop : 0; /* hand over to our known next hop */
    t->buf[15] = my_byte;
    t->is_relay = true;
    t->from = h.from;
    t->id = h.id;
    t->to = h.to;
    seen->relayed = true;
    /* SNR-weighted contention window: weaker links relay first (Meshtastic). */
    uint32_t slot = mesh_slot_ms(s_preset);
    float s = snr < -20 ? -20 : snr > 10 ? 10 : snr;
    uint32_t cw = 3 + (uint32_t)((s + 20) * 5 / 30); /* 3..8 */
    int64_t base = now_ms();
    if (role == EDGE_ROLE_ROUTER || role == EDGE_ROLE_REPEATER) {
        t->not_before = base + rnd(2 * cw) * slot;
    } else if (role == EDGE_ROLE_ROUTER_LATE) {
        t->not_before = base + 2 * 8 * slot + (1u << 8) * slot + rnd(1u << cw) * slot;
        t->late = true;
    } else {
        t->not_before = base + 2 * 8 * slot + rnd(1u << cw) * slot;
    }
}

/* ---- Persistence ("meshdb" partition, two alternating 32 KB slots) ---------------------- */

#define DB_MAGIC 0x3248534d /* "MSH2" */
#define DB_SLOT  0x8000

typedef struct {
    uint32_t magic;
    uint32_t gen;
    uint32_t len;
    uint32_t crc;
    uint32_t node_size, msg_size, wp_size;
} db_hdr_t;

static const esp_partition_t *db_part(void)
{
    return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x41, "meshdb");
}

static uint32_t db_crc(void)
{
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)s_nodes, MAX_NODES * sizeof(node_t));
    crc = esp_rom_crc32_le(crc, (const uint8_t *)s_msgs, MAX_MSGS * sizeof(msg_t));
    return esp_rom_crc32_le(crc, (const uint8_t *)s_wps, MAX_WAYPOINTS * sizeof(wp_t));
}

static void db_save(void)
{
    const esp_partition_t *p = db_part();
    if (!p || !s_nodes) return;
    size_t body = MAX_NODES * sizeof(node_t) + MAX_MSGS * sizeof(msg_t) + MAX_WAYPOINTS * sizeof(wp_t);
    if (sizeof(db_hdr_t) + body > DB_SLOT) {
        ESP_LOGE(TAG, "mesh state too large to save");
        return;
    }
    db_hdr_t a = {0}, b = {0};
    esp_partition_read(p, 0, &a, sizeof(a));
    esp_partition_read(p, DB_SLOT, &b, sizeof(b));
    uint32_t ga = a.magic == DB_MAGIC ? a.gen : 0, gb = b.magic == DB_MAGIC ? b.gen : 0;
    size_t off = ga <= gb ? 0 : DB_SLOT; /* overwrite the older slot */
    for (int i = 0; i < MAX_NODES; i++) { /* never persist derived keys */
        s_nodes[i].key_ok = false;
        memset(s_nodes[i].shared, 0, 32);
    }
    db_hdr_t h = {.magic = DB_MAGIC, .gen = (ga > gb ? ga : gb) + 1, .len = (uint32_t)body, .crc = db_crc(),
                  .node_size = sizeof(node_t), .msg_size = sizeof(msg_t), .wp_size = sizeof(wp_t)};
    if (esp_partition_erase_range(p, off, DB_SLOT) != ESP_OK) return;
    size_t o = off + sizeof(h);
    esp_partition_write(p, o, s_nodes, MAX_NODES * sizeof(node_t));
    o += MAX_NODES * sizeof(node_t);
    esp_partition_write(p, o, s_msgs, MAX_MSGS * sizeof(msg_t));
    o += MAX_MSGS * sizeof(msg_t);
    esp_partition_write(p, o, s_wps, MAX_WAYPOINTS * sizeof(wp_t));
    esp_partition_write(p, off, &h, sizeof(h)); /* header last: a torn write leaves the other slot newest */
    s_dirty = false;
    ESP_LOGI(TAG, "saved %d nodes", node_count());
}

static bool db_read_slot(const esp_partition_t *p, size_t off, const db_hdr_t *h)
{
    if (h->magic != DB_MAGIC || h->node_size != sizeof(node_t) || h->msg_size != sizeof(msg_t) ||
        h->wp_size != sizeof(wp_t)) {
        return false;
    }
    size_t o = off + sizeof(db_hdr_t);
    esp_partition_read(p, o, s_nodes, MAX_NODES * sizeof(node_t));
    o += MAX_NODES * sizeof(node_t);
    esp_partition_read(p, o, s_msgs, MAX_MSGS * sizeof(msg_t));
    o += MAX_MSGS * sizeof(msg_t);
    esp_partition_read(p, o, s_wps, MAX_WAYPOINTS * sizeof(wp_t));
    if (db_crc() == h->crc) return true;
    memset(s_nodes, 0, MAX_NODES * sizeof(node_t));
    memset(s_msgs, 0, MAX_MSGS * sizeof(msg_t));
    memset(s_wps, 0, MAX_WAYPOINTS * sizeof(wp_t));
    return false;
}

static void db_load(void)
{
    const esp_partition_t *p = db_part();
    if (!p) return;
    db_hdr_t a = {0}, b = {0};
    esp_partition_read(p, 0, &a, sizeof(a));
    esp_partition_read(p, DB_SLOT, &b, sizeof(b));
    bool a_first = a.magic == DB_MAGIC && (b.magic != DB_MAGIC || a.gen >= b.gen);
    bool ok = a_first ? (db_read_slot(p, 0, &a) || db_read_slot(p, DB_SLOT, &b))
                      : (db_read_slot(p, DB_SLOT, &b) || db_read_slot(p, 0, &a));
    if (!ok) return;
    /* Times from before this boot: convert to this boot's uptime when the clock is known. */
    int64_t up = now_ms();
    uint32_t nowu = now_unix_s();
    for (int i = 0; i < MAX_NODES; i++) {
        node_t *n = &s_nodes[i];
        n->key_ok = false;
        n->last_nodeinfo_reply = 0;
        n->last_heard_up = (nowu && n->last_heard_s && n->last_heard_s <= nowu) ? up - (int64_t)(nowu - n->last_heard_s) * 1000 : 0;
        if (n->num && is_ignored(n->num)) n->flags |= NF_IGNORE;
    }
    int64_t newest = -1;
    for (int i = 0; i < MAX_MSGS; i++) {
        msg_t *m = &s_msgs[i];
        m->at = (nowu && m->at_s && m->at_s <= nowu) ? up - (int64_t)(nowu - m->at_s) * 1000 : 0;
        if (m->used && (int64_t)m->at_s >= newest) {
            newest = m->at_s;
            s_msg_next = (i + 1) % MAX_MSGS;
        }
    }
    ESP_LOGI(TAG, "restored %d nodes", node_count());
}

/* ---- Peripherals ---------------------------------------------------------------------- */

static void periph_start(void)
{
    const edge_mesh_ext_t *x = &s_cfg.mx;
    if (x->serial_enabled && x->serial_rx >= 0 && x->serial_tx >= 0 && !s_serial_on) {
        uart_config_t uc = {.baud_rate = (int)x->serial_baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                            .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
        if (uart_driver_install(UART_NUM_1, 512, 0, 0, NULL, 0) == ESP_OK) {
            uart_param_config(UART_NUM_1, &uc);
            uart_set_pin(UART_NUM_1, x->serial_tx, x->serial_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            s_serial_on = true;
            s_serial_len = 0;
        }
    }
    if (x->detect_enabled && x->detect_gpio >= 0 && !s_detect_on) {
        gpio_config_t g = {.pin_bit_mask = 1ULL << x->detect_gpio, .mode = GPIO_MODE_INPUT,
                           .pull_up_en = x->detect_active_high ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
                           .pull_down_en = x->detect_active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE};
        gpio_config(&g);
        s_detect_last = gpio_get_level(x->detect_gpio);
        s_detect_on = true;
    }
    if (x->notify_gpio >= 0) {
        gpio_config_t g = {.pin_bit_mask = 1ULL << x->notify_gpio, .mode = GPIO_MODE_OUTPUT};
        gpio_config(&g);
        gpio_set_level(x->notify_gpio, 0);
    }
    if (x->pax_enabled) {
        portENTER_CRITICAL(&s_pax_mux);
        s_pax_nb = s_pax_nw = 0;
        portEXIT_CRITICAL(&s_pax_mux);
        s_pax_on = true;
        s_pax_reassert = now_ms() + 2000; /* after the awareness pipeline has released the radios */
    }
}

static void periph_stop(void)
{
    if (s_serial_on) {
        uart_driver_delete(UART_NUM_1);
        s_serial_on = false;
    }
    s_detect_on = false;
    if (s_pax_on) {
        s_pax_on = false;
        edge_ble_scan_stop();
        edge_wifi_sniff_set(false, false);
    }
    if (s_cfg.mx.notify_gpio >= 0) gpio_set_level(s_cfg.mx.notify_gpio, 0);
    s_notify_until = 0;
}

void mesh_pax_seen(bool wifi, const uint8_t mac[6])
{
    if (!s_pax_on) return;
    uint32_t h = edge_fnv1a(NULL, 0, mac, 6);
    portENTER_CRITICAL(&s_pax_mux);
    uint32_t *arr = wifi ? s_pax_wifi : s_pax_ble;
    int *n = wifi ? &s_pax_nw : &s_pax_nb;
    bool found = false;
    for (int i = 0; i < *n && !found; i++) found = arr[i] == h;
    if (!found && *n < PAX_SLOTS) arr[(*n)++] = h;
    portEXIT_CRITICAL(&s_pax_mux);
}

static void periph_poll(void)
{
    const edge_mesh_ext_t *x = &s_cfg.mx;
    int64_t t = now_ms();
    if (s_serial_on) {
        uint8_t b[64];
        int n = uart_read_bytes(UART_NUM_1, b, sizeof(b), 0);
        for (int i = 0; i < n; i++) {
            if (b[i] == '\r') continue;
            if (b[i] != '\n' && s_serial_len < MAX_TEXT) {
                s_serial_line[s_serial_len++] = (char)b[i];
                continue;
            }
            s_serial_line[s_serial_len] = '\0';
            if (s_serial_len) {
                mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)s_serial_line, .payload_len = s_serial_len};
                send_data(MT_BROADCAST, 0, &d, false, false, s_cfg.hop_limit, NULL, NULL);
            }
            s_serial_len = 0;
        }
    }
    if (s_detect_on) {
        int lvl = gpio_get_level(x->detect_gpio);
        bool active = x->detect_active_high ? lvl == 1 : lvl == 0;
        bool was = x->detect_active_high ? s_detect_last == 1 : s_detect_last == 0;
        s_detect_last = lvl;
        if (active && !was && t - s_detect_sent > (int64_t)x->detect_min_interval_s * 1000) {
            s_detect_sent = t;
            char msg[48];
            int l = snprintf(msg, sizeof(msg), "%s detected", x->detect_name);
            send_port(MT_BROADCAST, 0, MT_PORT_DETECTION, (const uint8_t *)msg, (size_t)l, false, 0, s_cfg.hop_limit);
        }
    }
    if (x->notify_gpio >= 0) {
        bool on = t < s_notify_until && ((t / 250) % 2 == 0);
        gpio_set_level(x->notify_gpio, on ? 1 : 0);
    }
    if (s_pax_on && s_pax_reassert && t >= s_pax_reassert) {
        s_pax_reassert = 0;
        edge_ble_scan_start(false);       /* passive: counting only, nothing is logged */
        edge_wifi_sniff_set(false, true);
    }
}

bool mesh_led(int64_t t, bool *on)
{
    if (t >= s_notify_until) return false;
    *on = (t / 250) % 2 == 0;
    return true;
}

/* ---- Radio control -------------------------------------------------------------------- */

static void apply_config(void)
{
    edge_config_get(&s_cfg);
    edge_mod_get(&s_mod);
    s_me = s_cfg.node_num;
    s_have_keys = edge_pki_get(s_priv, s_pub) == ESP_OK;
    s_sfs_on = s_mod.sf_enabled && sfdb_ready();
    s_mqtt_restart = true;
    s_mqtt_public = strstr(s_mod.mqtt_address, "mqtt.meshtastic.org") != NULL;
    s_region = mesh_region_by_code(s_cfg.region);
    s_preset = mesh_preset_by_code(s_cfg.preset);
    if (!s_preset) s_preset = mesh_preset_by_code(0);
    s_nch = s_cfg.n_channels;
    for (int i = 0; i < s_nch; i++) {
        const edge_mesh_channel_t *c = &s_cfg.channels[i];
        edge_strlcpy(s_ch[i].name, mesh_effective_channel_name(c->name, s_preset), sizeof(s_ch[i].name));
        /* Licensed (amateur) operation sends without encryption. */
        s_ch[i].key_len = s_cfg.mx.licensed ? 0 : mesh_expand_psk(c->psk, c->psk_len, s_ch[i].key);
        s_ch[i].hash = mesh_channel_hash(s_ch[i].name, s_ch[i].key, s_ch[i].key_len);
        s_ch[i].precision = c->position_precision;
    }
    for (int i = 0; s_nodes && i < MAX_NODES; i++) {
        s_nodes[i].key_ok = false;
        if (!s_nodes[i].num) continue;
        if (is_ignored(s_nodes[i].num)) {
            s_nodes[i].flags |= NF_IGNORE;
        } else {
            s_nodes[i].flags &= (uint8_t)~NF_IGNORE;
        }
    }
    if (s_mod.mqtt_root[0]) {
        edge_strlcpy(s_mqtt_root, s_mod.mqtt_root, sizeof(s_mqtt_root));
    } else {
        snprintf(s_mqtt_root, sizeof(s_mqtt_root), "msh/%s", s_region ? s_region->name : "UNSET");
    }
    if (s_rhw_watch && !s_mod.rhw_enabled) s_rhw_watch = 0;
    if (!s_region) {
        s_freq = 0;
        ESP_LOGW(TAG, "LoRa region not set: radio stays silent");
        return;
    }
    s_freq = s_cfg.mx.override_freq_khz ? s_cfg.mx.override_freq_khz / 1000.0f
                                        : mesh_frequency_mhz(s_region, s_preset, s_cfg.channels[0].name, s_cfg.channel_num);
    s_freq += s_cfg.mx.freq_offset_hz / 1e6f;
    int pwr = s_region->power_limit;
    if (s_cfg.tx_power > 0 && s_cfg.tx_power < pwr) pwr = s_cfg.tx_power;
    if (pwr > CONFIG_EDGE_LORA_MAX_POWER_DBM) pwr = CONFIG_EDGE_LORA_MAX_POWER_DBM;
    sx1262_cfg_t rc = {
        .freq_mhz = s_freq,
        .bw_khz = s_preset->bw_khz,
        .sf = s_preset->sf,
        .cr = s_preset->cr,
        .power_dbm = (int8_t)pwr,
        .preamble = MESH_PREAMBLE_LEN,
        .sync_word = MESH_SYNC_WORD,
        .rx_boosted = s_cfg.mx.rx_boosted,
    };
    if (sx1262_configure(&rc) == ESP_OK) sx1262_start_rx();
    ESP_LOGI(TAG, "node !%08lx (%s) on %s %s, %.3f MHz", (unsigned long)s_me, role_name(s_cfg.mx.role), s_region->name,
             s_preset->display, s_freq);
}

static void free_state(void)
{
    free(s_txq);
    free(s_pending);
    free(s_seen);
    free(s_nodes);
    free(s_msgs);
    free(s_wps);
    free(s_tak);
    s_tak = NULL;
    s_txq = NULL;
    s_pending = NULL;
    s_seen = NULL;
    s_nodes = NULL;
    s_msgs = NULL;
    s_wps = NULL;
}

static void do_start(void)
{
    if (!s_radio_ok || s_running) return;
    s_txq = calloc(TXQ_LEN, sizeof(tx_item_t));
    s_pending = calloc(MAX_PENDING, sizeof(pending_t));
    s_seen = calloc(SEEN_LEN, sizeof(seen_t));
    s_nodes = calloc(MAX_NODES, sizeof(node_t));
    s_msgs = calloc(MAX_MSGS, sizeof(msg_t));
    s_wps = calloc(MAX_WAYPOINTS, sizeof(wp_t));
    s_tak = calloc(MAX_TAK, sizeof(tak_t));
    if (!s_txq || !s_pending || !s_seen || !s_nodes || !s_msgs || !s_wps || !s_tak) {
        ESP_LOGE(TAG, "not enough memory for the mesh");
        free_state();
        return;
    }
    s_seen_next = s_msg_next = 0;
    edge_config_get(&s_cfg);
    db_load();
    s_tx_busy = false;
    s_running = true;
    apply_config();
    periph_start();
    int64_t t = now_ms();
    s_next_nodeinfo = t + 5000 + rnd(10000);
    s_next_position = t + 20000 + rnd(10000);
    s_next_telemetry = t + 60000 + rnd(30000);
    s_next_neighbor = t + 120000;
    s_next_range = t + 15000;
    s_next_pax = t + (int64_t)s_cfg.mx.pax_interval_s * 1000;
    s_next_save = t + SAVE_EVERY_MS;
    s_next_lost = t + 30000;
}

static void do_stop(void)
{
    if (!s_running) return;
    periph_stop();
    mesh_mqtt_stop();
    if (s_dirty) db_save();
    s_running = false;
    s_tx_busy = false;
    if (s_radio_ok) sx1262_sleep();
    free_state();
}

static bool channel_busy(void)
{
    if (!s_rx_activity) return false;
    if (now_ms() - s_rx_activity > (int64_t)mesh_airtime_ms(s_preset, MESH_MAX_PACKET)) {
        s_rx_activity = 0;
        return false;
    }
    return true;
}

static void handle_irq(void)
{
    for (int guard = 0; guard < 8; guard++) {
        uint16_t irq = sx1262_get_irq();
        if (!irq) return;
        sx1262_clear_irq(irq);
        if (irq & (SX_IRQ_PREAMBLE | SX_IRQ_HEADER_VALID)) s_rx_activity = now_ms();
        if (irq & SX_IRQ_PREAMBLE) s_st.preambles++;
        if (irq & SX_IRQ_HEADER_VALID) s_st.headers++;
        if (irq & (SX_IRQ_CRC_ERR | SX_IRQ_HEADER_ERR)) {
            s_rx_activity = 0;
            s_st.rx_bad++;
        } else if (irq & SX_IRQ_RX_DONE) {
            s_rx_activity = 0;
            uint8_t buf[MESH_MAX_PACKET];
            size_t len = 0;
            int16_t rssi = 0;
            float snr = 0;
            if (sx1262_read_packet(buf, sizeof(buf), &len, &rssi, &snr) == ESP_OK) process_rx(buf, len, rssi, snr);
        }
        if ((irq & SX_IRQ_TX_DONE) && s_tx_busy) {
            s_tx_busy = false;
            s_st.tx_ok++;
            air_add(s_tx_airtime, true);
            if (s_tx_cur.is_relay) {
                s_st.relayed++;
            } else if (s_tx_cur.track) {
                bool bcast = s_tx_cur.to == MT_BROADCAST;
                bool want_ack = (s_tx_cur.buf[12] & MT_FLAG_WANT_ACK) != 0;
                if (s_tx_cur.tries == 1) publish_tx_status(s_tx_cur.id, "sent", -1);
                if (want_ack || bcast) {
                    pending_t *p = NULL;
                    for (int i = 0; i < MAX_PENDING; i++) {
                        if (s_pending[i].used && s_pending[i].id == s_tx_cur.id) p = &s_pending[i];
                    }
                    for (int i = 0; i < MAX_PENDING && !p; i++) {
                        if (!s_pending[i].used) p = &s_pending[i];
                    }
                    if (p) {
                        p->used = true;
                        p->broadcast = bcast;
                        p->id = s_tx_cur.id;
                        p->to = s_tx_cur.to;
                        p->tries = s_tx_cur.tries;
                        p->len = s_tx_cur.len;
                        memcpy(p->buf, s_tx_cur.buf, s_tx_cur.len);
                        p->deadline = now_ms() + 4 * (int64_t)s_tx_airtime * s_cfg.hop_limit + 5000;
                    }
                }
            }
            sx1262_start_rx();
        }
    }
}

static void service_tx(void)
{
    int64_t t = now_ms();
    if (s_tx_busy) {
        if (t - s_tx_started > (int64_t)s_tx_airtime + 3000) {
            ESP_LOGW(TAG, "transmit timed out, returning to receive");
            s_tx_busy = false;
            sx1262_start_rx();
        }
        return;
    }
    tx_item_t *best = NULL;
    for (int i = 0; i < TXQ_LEN; i++) {
        if (s_txq[i].used && s_txq[i].not_before <= t && (!best || s_txq[i].not_before < best->not_before)) best = &s_txq[i];
    }
    if (!best) return;
    if (!s_cfg.mx.tx_enabled) {
        best->used = false;
        return;
    }
    if (channel_busy()) {
        best->not_before = t + (1 + rnd(4)) * mesh_slot_ms(s_preset);
        return;
    }
    uint32_t air = mesh_airtime_ms(s_preset, best->len);
    if (!duty_allows(air)) {
        if (best->is_relay) {
            best->used = false;
            s_st.tx_dropped++;
        } else {
            best->not_before = t + 30000;
        }
        return;
    }
    s_tx_cur = *best;
    s_tx_cur.tries++;
    best->used = false;
    if (sx1262_transmit(s_tx_cur.buf, s_tx_cur.len) == ESP_OK) {
        s_tx_busy = true;
        s_tx_started = t;
        s_tx_airtime = air;
    } else {
        sx1262_start_rx();
    }
}

static void service_pending(void)
{
    int64_t t = now_ms();
    for (int i = 0; i < MAX_PENDING; i++) {
        pending_t *p = &s_pending[i];
        if (!p->used || t < p->deadline) continue;
        if (!p->broadcast && p->tries < ACK_RETRIES) {
            tx_item_t *q = txq_alloc();
            if (q) {
                memcpy(q->buf, p->buf, p->len);
                q->len = p->len;
                if (p->tries == ACK_RETRIES - 1) {
                    q->buf[14] = 0; /* last try floods instead of trusting the learned next hop */
                    node_t *dest = node_get(p->to, false);
                    if (dest) dest->next_hop = 0;
                }
                q->from = s_me;
                q->id = p->id;
                q->to = p->to;
                q->track = true;
                q->tries = p->tries;
                q->not_before = t + rnd(8) * mesh_slot_ms(s_preset);
            }
            p->used = false;
        } else {
            p->used = false;
            publish_tx_status(p->id, p->broadcast ? "no_relay_heard" : "no_ack", -1);
        }
    }
}

/* Own periodic broadcasts, by role, politely. */
static void service_periodic(void)
{
    int64_t t = now_ms();
    const edge_mesh_ext_t *x = &s_cfg.mx;
    uint8_t role = x->role;
    bool hidden = role == EDGE_ROLE_CLIENT_HIDDEN;
    bool polite = polite_allows();
    int32_t la, lo, al;
    bool have_pos = own_position(&la, &lo, &al);

    if (t >= s_next_nodeinfo) {
        if (polite && !hidden && role != EDGE_ROLE_REPEATER && s_cfg.nodeinfo_interval_s) send_nodeinfo(MT_BROADCAST, false);
        s_next_nodeinfo = t + (polite ? (int64_t)(s_cfg.nodeinfo_interval_s ? s_cfg.nodeinfo_interval_s : 3600) * 1000 : 60000);
    }
    if (t >= s_next_position) {
        if (polite && have_pos && !hidden && role != EDGE_ROLE_REPEATER && s_ch[0].precision) {
            /* TAK trackers report as a TAK position (ATAK plugin v2), like Meshtastic */
            if (role == EDGE_ROLE_TAK_TRACKER) tak_send_pli(0);
            else send_position(MT_BROADCAST, 0, false, 0);
        }
        uint32_t every = x->pos_interval_s;
        if ((role == EDGE_ROLE_TRACKER || role == EDGE_ROLE_TAK_TRACKER) && every > 300) every = 300;
        s_next_position = t + (polite ? (int64_t)every * 1000 : 60000);
    }
    if (t >= s_next_telemetry) {
        if (polite && x->telemetry_interval_s && !hidden && role != EDGE_ROLE_REPEATER) send_telemetry(MT_BROADCAST, 0);
        uint32_t every = x->telemetry_interval_s ? x->telemetry_interval_s : 1800;
        if (role == EDGE_ROLE_SENSOR && every > 600) every = 600;
        s_next_telemetry = t + (polite ? (int64_t)every * 1000 : 60000);
    }
    if (t >= s_next_neighbor) {
        if (polite && x->neighbor_info && !hidden) send_neighbor_info();
        s_next_neighbor = t + (int64_t)x->neighbor_interval_s * 1000;
    }
    if (x->range_test_interval_s && t >= s_next_range) {
        char msg[24];
        int l = snprintf(msg, sizeof(msg), "seq %lu", (unsigned long)++s_range_seq);
        send_port(MT_BROADCAST, 0, MT_PORT_RANGE_TEST, (const uint8_t *)msg, (size_t)l, false, 0, s_cfg.hop_limit);
        s_next_range = t + (int64_t)x->range_test_interval_s * 1000;
    }
    if (s_pax_on && t >= s_next_pax) {
        uint32_t nb, nw;
        portENTER_CRITICAL(&s_pax_mux);
        nb = (uint32_t)s_pax_nb;
        nw = (uint32_t)s_pax_nw;
        s_pax_nb = s_pax_nw = 0;
        portEXIT_CRITICAL(&s_pax_mux);
        if (polite) {
            uint8_t pb[24];
            size_t pl = mt_paxcount_encode(nw, nb, (uint32_t)(t / 1000), pb, sizeof(pb));
            send_port(MT_BROADCAST, 0, MT_PORT_PAXCOUNTER, pb, pl, false, 0, s_cfg.hop_limit);
        }
        s_next_pax = t + (int64_t)x->pax_interval_s * 1000;
    }
    if (role == EDGE_ROLE_LOST_AND_FOUND && t >= s_next_lost) {
        if (have_pos) {
            char msg[80];
            int l = snprintf(msg, sizeof(msg), "I'm lost! Lat: %.5f Lon: %.5f", la / 1e7, lo / 1e7);
            mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)msg, .payload_len = (size_t)l};
            send_data(MT_BROADCAST, 0, &d, false, false, s_cfg.hop_limit, NULL, NULL);
            send_position(MT_BROADCAST, 0, false, 0);
        }
        s_next_lost = t + (int64_t)x->pos_interval_s * 1000;
    }
    sfs_service(t, polite);
    rhw_poll(t);
    mqtt_manage();
    if (s_reload_at && t >= s_reload_at) {
        s_reload_at = 0;
        periph_stop();
        apply_config();
        periph_start();
        if (s_wifi_reconf) {
            s_wifi_reconf = false;
            edge_wifi_reconfigure();
        }
    }
    if (s_nodeinfo_at && t >= s_nodeinfo_at) {
        s_nodeinfo_at = 0;
        send_nodeinfo(MT_BROADCAST, false);
    }
    uint32_t nowu = now_unix_s();
    for (int i = 0; nowu && i < MAX_WAYPOINTS; i++) {
        if (s_wps[i].used && s_wps[i].w.expire && s_wps[i].w.expire < nowu) s_wps[i].used = false;
    }
    if (s_dirty && t >= s_next_save) {
        db_save();
        s_next_save = t + SAVE_EVERY_MS;
    }
}

static uint32_t next_wait_ms(void)
{
    int64_t t = now_ms();
    int64_t wait = 100;
    for (int i = 0; i < TXQ_LEN; i++) {
        if (s_txq[i].used) {
            int64_t w = s_txq[i].not_before - t;
            if (w < wait) wait = w;
        }
    }
    return wait < 1 ? 1 : (uint32_t)wait;
}

static void mesh_task(void *arg)
{
    esp_err_t err = sx1262_init(xTaskGetCurrentTaskHandle());
    s_radio_ok = err == ESP_OK;
    if (s_radio_ok) {
        sx1262_sleep();
    } else {
        ESP_LOGE(TAG, "LoRa radio unavailable: %s", esp_err_to_name(err));
    }
    if (s_cfg.mode != EDGE_MODE_MESH) {
        /* Awareness boot: the radio sleeps and this task's memory goes back to sensing.
         * Switching to Mesh restarts the Edge. */
        s_task = NULL;
        vTaskDelete(NULL);
    }
    for (;;) {
        uint32_t wait = s_running ? next_wait_ms() : 1000;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t req = __atomic_exchange_n(&s_req, 0, __ATOMIC_SEQ_CST);
        if (req & REQ_STOP) do_stop();
        if (req & REQ_START) do_start();
        if ((req & REQ_RELOAD) && s_running) {
            periph_stop();
            memset(s_txq, 0, TXQ_LEN * sizeof(tx_item_t));
            s_tx_busy = false;
            apply_config();
            periph_start();
        }
        if ((req & REQ_SAVE) && s_running && s_dirty) db_save();
        if (s_running) {
            periph_poll();
            if (s_region) {
                handle_irq();
                if (sx1262_dio1_high()) handle_irq();
                service_pending();
                service_tx();
                service_periodic();
            }
        }
        xSemaphoreGive(s_lock);
        if (s_reboot_at && now_ms() >= s_reboot_at) {
            s_reboot_at = 0;
            edge_app_reboot_later(200);
        }
    }
}

/* ---- Public API ---------------------------------------------------------------------- */

esp_err_t mesh_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    edge_config_get(&s_cfg);
    edge_mod_get(&s_mod);
    s_me = s_cfg.node_num;
    if (s_cfg.mode == EDGE_MODE_MESH) sfdb_init();
    if (xTaskCreatePinnedToCore(mesh_task, "mesh", 7168, NULL, 6, &s_task, 1) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static void request(uint32_t bit)
{
    __atomic_or_fetch(&s_req, bit, __ATOMIC_SEQ_CST);
    if (s_task) xTaskNotifyGive(s_task);
}

void mesh_start(void) { request(REQ_START); }
void mesh_stop(void) { request(REQ_STOP); }
void mesh_reload(void) { request(REQ_RELOAD); }
bool mesh_running(void) { return s_running; }

void mesh_save_now(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_running && s_dirty) db_save();
    xSemaphoreGive(s_lock);
}

static esp_err_t running_ok(void)
{
    return s_running && s_region ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t mesh_send_text(uint32_t to, uint8_t channel, const char *text, bool want_ack, uint32_t reply_id, uint32_t emoji,
                         uint32_t *out_id)
{
    if (!text) return ESP_ERR_INVALID_ARG;
    size_t len = strlen(text);
    if (len == 0 || len > MAX_TEXT) return ESP_ERR_INVALID_SIZE;
    if (to == 0) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    uint32_t id = 0;
    bool pki = false;
    if (err == ESP_OK) {
        mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)text, .payload_len = len,
                       .reply_id = reply_id, .emoji = emoji};
        bool ack = want_ack && to != MT_BROADCAST;
        err = send_data(to, channel, &d, ack, true, s_cfg.hop_limit, &id, &pki);
    }
    if (err == ESP_OK) {
        msg_t *m = msg_add();
        m->outgoing = true;
        m->pki = pki;
        m->id = id;
        m->from = s_me;
        m->to = to;
        m->channel = channel;
        m->reply_id = reply_id;
        m->emoji = emoji;
        edge_strlcpy(m->status, "queued", sizeof(m->status));
        edge_sanitize_text(m->text, sizeof(m->text), (const uint8_t *)text, len);
        if (to == MT_BROADCAST && !pki) {
            mt_data_t sd = {.reply_id = reply_id, .emoji = emoji};
            sfs_store(s_me, to, id, channel, &sd, (const uint8_t *)text, len);
        }
        log_text(s_me, to, id, channel, s_cfg.hop_limit, 0, 0, true, text, len);
        if (out_id) *out_id = id;
    }
    xSemaphoreGive(s_lock);
    return err;
}

int mesh_canned_count(void)
{
    edge_config_t c;
    edge_config_get(&c);
    return c.mx.n_canned;
}

bool mesh_canned_get(int i, char *out, size_t cap)
{
    edge_config_t c;
    edge_config_get(&c);
    if (i < 0 || i >= c.mx.n_canned) return false;
    edge_strlcpy(out, c.mx.canned[i], cap);
    return true;
}

esp_err_t mesh_send_canned(int i, uint32_t *out_id)
{
    char t[40];
    if (!mesh_canned_get(i, t, sizeof(t)) || !t[0]) return ESP_ERR_NOT_FOUND;
    return mesh_send_text(MT_BROADCAST, 0, t, false, 0, 0, out_id);
}

esp_err_t mesh_traceroute(uint32_t to, uint8_t channel, uint32_t *out_id)
{
    if (to == 0 || to == MT_BROADCAST) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) {
        mt_route_t r = {0};
        uint8_t rb[16];
        size_t rl = mt_route_encode(&r, rb, sizeof(rb));
        mt_data_t d = {.portnum = MT_PORT_TRACEROUTE, .payload = rb, .payload_len = rl, .want_response = true};
        err = send_data(to, channel, &d, false, false, s_cfg.hop_limit, out_id, NULL);
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t mesh_request_position(uint32_t to, uint8_t channel)
{
    if (to == 0 || to == MT_BROADCAST) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) send_position(to, channel < s_nch ? channel : 0, true, 0);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t mesh_send_waypoint(const mt_waypoint_t *w_in, uint8_t channel, uint32_t *out_id)
{
    mt_waypoint_t w = *w_in;
    if (!w.id) w.id = esp_random();
    uint8_t buf[200];
    size_t l = mt_waypoint_encode(&w, buf, sizeof(buf));
    if (!l) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) {
        mt_data_t d = {.portnum = MT_PORT_WAYPOINT, .payload = buf, .payload_len = l};
        err = send_data(MT_BROADCAST, channel, &d, false, false, s_cfg.hop_limit, NULL, NULL);
        if (err == ESP_OK && s_wps) {
            wp_t *slot = NULL;
            for (int i = 0; i < MAX_WAYPOINTS && !slot; i++) {
                if (!s_wps[i].used || s_wps[i].w.id == w.id) slot = &s_wps[i];
            }
            if (slot) {
                slot->used = true;
                slot->from = s_me;
                slot->at_s = now_unix_s();
                slot->w = w;
                s_dirty = true;
            }
        }
    }
    xSemaphoreGive(s_lock);
    if (out_id) *out_id = w.id;
    return err;
}

esp_err_t mesh_sf_request(uint32_t router, uint8_t channel)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) {
        if (!router) {
            int64_t best = -1;
            for (int i = 0; i < MAX_SF_ROUTERS; i++) {
                if (s_sf[i].num && s_sf[i].at > best) {
                    best = s_sf[i].at;
                    router = s_sf[i].num;
                }
            }
        }
        if (!router) {
            err = ESP_ERR_NOT_FOUND;
        } else {
            mt_sf_t sf = {.rr = MT_SF_CLIENT_HISTORY, .has_history = true, .window = 120};
            uint8_t b[24];
            size_t l = mt_sf_encode(&sf, b, sizeof(b));
            err = send_port(router, channel, MT_PORT_STORE_FWD, b, l, false, 0, s_cfg.hop_limit);
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t mesh_send_nodeinfo_now(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) send_nodeinfo(MT_BROADCAST, true);
    xSemaphoreGive(s_lock);
    return err;
}

void mesh_set_client_position(int32_t lat_i, int32_t lon_i, int32_t alt)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_client_lat = lat_i;
    s_client_lon = lon_i;
    s_client_alt = alt;
    s_client_pos = true;
    xSemaphoreGive(s_lock);
}

esp_err_t mesh_node_flags(uint32_t num, int favorite, int ignored)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    node_t *n = node_get(num, false);
    if (n) {
        if (favorite >= 0) n->flags = (uint8_t)(favorite ? n->flags | NF_FAV : n->flags & ~NF_FAV);
        if (ignored >= 0) n->flags = (uint8_t)(ignored ? n->flags | NF_IGNORE : n->flags & ~NF_IGNORE);
        s_dirty = true;
    }
    xSemaphoreGive(s_lock);
    return n ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void mesh_get_summary(mesh_summary_t *o)
{
    memset(o, 0, sizeof(*o));
    xSemaphoreTake(s_lock, portMAX_DELAY);
    o->radio_present = s_radio_ok;
    o->running = s_running;
    o->region_set = s_region != NULL;
    o->region = s_region ? s_region->name : NULL;
    o->preset = s_preset ? s_preset->display : NULL;
    o->freq_mhz = s_freq;
    o->node_num = s_me;
    o->nodes = node_count();
    o->rx_packets = s_st.rx_ok;
    o->tx_packets = s_st.tx_ok;
    o->relayed = s_st.relayed;
    o->channel_util = s_running ? channel_util_pct() : 0;
    for (int k = 0; s_msgs && k < MAX_MSGS; k++) {
        msg_t *m = &s_msgs[(s_msg_next + MAX_MSGS - 1 - k) % MAX_MSGS];
        if (m->used && !m->outgoing) {
            node_t *n = node_get(m->from, false);
            if (n && (n->flags & NF_USER) && n->short_name[0]) {
                edge_strlcpy(o->last_from, n->short_name, sizeof(o->last_from));
            } else {
                snprintf(o->last_from, sizeof(o->last_from), "%04lx", (unsigned long)(m->from & 0xffff));
            }
            edge_strlcpy(o->last_text, m->text, sizeof(o->last_text));
            break;
        }
    }
    xSemaphoreGive(s_lock);
}

cJSON *mesh_status_json(void)
{
    cJSON *o = cJSON_CreateObject();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    char id[12];
    node_id_str(s_me, id);
    cJSON_AddBoolToObject(o, "radio_present", s_radio_ok);
    cJSON_AddBoolToObject(o, "running", s_running);
    cJSON_AddStringToObject(o, "node_id", id);
    cJSON_AddNumberToObject(o, "node_num", s_me);
    cJSON_AddStringToObject(o, "role", role_name(s_cfg.mx.role));
    if (s_have_keys) {
        char b64[48];
        edge_b64_encode(s_pub, 32, b64, sizeof(b64));
        cJSON_AddStringToObject(o, "public_key", b64);
    }
    if (s_region) {
        cJSON_AddStringToObject(o, "region", s_region->name);
        cJSON_AddNumberToObject(o, "freq_mhz", s_freq);
        cJSON_AddNumberToObject(o, "duty_cycle_pct", s_region->duty_cycle);
    } else {
        cJSON_AddNullToObject(o, "region");
    }
    if (s_preset) cJSON_AddStringToObject(o, "preset", s_preset->name);
    if (s_running) {
        cJSON_AddNumberToObject(o, "channel_util_pct", round(channel_util_pct() * 10) / 10);
        cJSON_AddNumberToObject(o, "air_util_tx_pct", round(air_util_tx_pct() * 100) / 100);
        cJSON_AddBoolToObject(o, "polite_hold", !polite_allows());
    }
    cJSON *chs = cJSON_AddArrayToObject(o, "channels");
    for (int i = 0; i < s_nch; i++) {
        cJSON *c = cJSON_CreateObject();
        cJSON_AddNumberToObject(c, "index", i);
        cJSON_AddStringToObject(c, "name", s_ch[i].name);
        cJSON_AddNumberToObject(c, "hash", s_ch[i].hash);
        cJSON_AddBoolToObject(c, "encrypted", s_ch[i].key_len > 0);
        cJSON_AddItemToArray(chs, c);
    }
    cJSON_AddNumberToObject(o, "nodes", node_count());
    cJSON *sf = cJSON_AddArrayToObject(o, "store_forward_routers");
    for (int i = 0; i < MAX_SF_ROUTERS; i++) {
        if (!s_sf[i].num) continue;
        node_id_str(s_sf[i].num, id);
        cJSON_AddItemToArray(sf, cJSON_CreateString(id));
    }
    cJSON *adm = cJSON_AddObjectToObject(o, "admin");
    cJSON_AddBoolToObject(adm, "enabled", s_mod.admin_enabled);
    cJSON_AddStringToObject(adm, "mode", s_mod.admin_mode == EDGE_ADMIN_KEYS ? "keys" : "open");
    cJSON_AddNumberToObject(adm, "keys", s_mod.n_admin_keys);
    cJSON_AddNumberToObject(adm, "requests", s_admin_seen);
    cJSON_AddNumberToObject(adm, "refused", s_admin_refused);
    cJSON *sfs = cJSON_AddObjectToObject(o, "store_forward_server");
    cJSON_AddBoolToObject(sfs, "enabled", s_sfs_on);
    cJSON_AddBoolToObject(sfs, "storage", sfdb_ready());
    cJSON_AddNumberToObject(sfs, "stored", sfdb_count());
    cJSON_AddNumberToObject(sfs, "capacity", sfdb_capacity());
    cJSON_AddNumberToObject(sfs, "history_requests", s_sfs_hist_requests);
    if (s_sfs.num) {
        node_id_str(s_sfs.num, id);
        cJSON_AddStringToObject(sfs, "sending_to", id);
    }
    cJSON_AddItemToObject(o, "mqtt", mesh_mqtt_status_json());
    if (s_rhw_watch || s_rhw_out) {
        cJSON *rh = cJSON_AddObjectToObject(o, "remote_hardware");
        cJSON_AddNumberToObject(rh, "outputs_mask", (double)s_rhw_out);
        cJSON_AddNumberToObject(rh, "watch_mask", (double)s_rhw_watch);
    }
    if (s_audio_frames) cJSON_AddNumberToObject(o, "audio_frames_heard", s_audio_frames);
    cJSON_AddNumberToObject(o, "tak_packets_heard", s_tak_rx);
    if (s_pax_on) {
        cJSON *p = cJSON_AddObjectToObject(o, "paxcounter");
        cJSON_AddNumberToObject(p, "ble", s_pax_nb);
        cJSON_AddNumberToObject(p, "wifi", s_pax_nw);
    }
    cJSON *st = cJSON_AddObjectToObject(o, "counters");
    cJSON_AddNumberToObject(st, "rx", s_st.rx_ok);
    cJSON_AddNumberToObject(st, "rx_pki", s_st.rx_pki);
    cJSON_AddNumberToObject(st, "rx_bad", s_st.rx_bad);
    cJSON_AddNumberToObject(st, "rx_other_channel", s_st.rx_undecoded);
    cJSON_AddNumberToObject(st, "rx_duplicate", s_st.rx_dupe);
    cJSON_AddNumberToObject(st, "rx_ignored", s_st.ignored);
    cJSON_AddNumberToObject(st, "tx", s_st.tx_ok);
    cJSON_AddNumberToObject(st, "relayed", s_st.relayed);
    cJSON_AddNumberToObject(st, "relay_skipped", s_st.relay_cancelled);
    cJSON_AddNumberToObject(st, "tx_dropped", s_st.tx_dropped);
    cJSON_AddNumberToObject(st, "preambles", s_st.preambles);
    cJSON_AddNumberToObject(st, "headers", s_st.headers);
    if (s_running && s_region && s_radio_ok) {
        static const char *MODES[] = {"?", "?", "standby_rc", "standby_xosc", "fs", "rx", "tx", "?"};
        cJSON *radio = cJSON_AddObjectToObject(o, "radio");
        cJSON_AddStringToObject(radio, "chip_mode", MODES[sx1262_chip_mode() & 7]);
        cJSON_AddNumberToObject(radio, "rssi_now", sx1262_rssi_now());
    }
    xSemaphoreGive(s_lock);
    return o;
}

cJSON *mesh_nodes_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; s_nodes && i < MAX_NODES; i++) {
        if (s_nodes[i].num) cJSON_AddItemToArray(a, node_json(&s_nodes[i]));
    }
    xSemaphoreGive(s_lock);
    return a;
}

cJSON *mesh_messages_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int k = 0; s_msgs && k < MAX_MSGS; k++) {
        msg_t *m = &s_msgs[(s_msg_next + k) % MAX_MSGS];
        if (m->used) cJSON_AddItemToArray(a, msg_json(m));
    }
    xSemaphoreGive(s_lock);
    return a;
}

cJSON *mesh_waypoints_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; s_wps && i < MAX_WAYPOINTS; i++) {
        const wp_t *w = &s_wps[i];
        if (!w->used) continue;
        cJSON *o = cJSON_CreateObject();
        char id[12];
        cJSON_AddNumberToObject(o, "id", w->w.id);
        node_id_str(w->from, id);
        cJSON_AddStringToObject(o, "from", id);
        char s[sizeof(w->w.description)];
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)w->w.name, strlen(w->w.name));
        cJSON_AddStringToObject(o, "name", s);
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)w->w.description, strlen(w->w.description));
        cJSON_AddStringToObject(o, "description", s);
        if (w->w.has_lat && w->w.has_lon) {
            cJSON_AddNumberToObject(o, "lat", w->w.lat_i / 1e7);
            cJSON_AddNumberToObject(o, "lon", w->w.lon_i / 1e7);
        }
        if (w->w.expire) cJSON_AddNumberToObject(o, "expire_unix_s", w->w.expire);
        if (w->w.icon) cJSON_AddNumberToObject(o, "icon", w->w.icon);
        if (w->at_s) cJSON_AddNumberToObject(o, "unix_s", w->at_s);
        cJSON_AddStringToObject(o, "provenance", "NETWORK");
        cJSON_AddItemToArray(a, o);
    }
    xSemaphoreGive(s_lock);
    return a;
}

/* ---- ATAK API ---------------------------------------------------------------------------- */

esp_err_t mesh_tak_send_chat(const char *text, uint8_t channel)
{
    if (!text || !text[0] || strlen(text) > MAX_TEXT) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK) {
        mt_tak_t t;
        tak_fill_self(&t);
        t.cot_type = MT_COT_B_T_F;
        t.has_chat = true;
        edge_strlcpy(t.chat, text, sizeof(t.chat));
        edge_strlcpy(t.chat_to, "All Chat Rooms", sizeof(t.chat_to));
        err = tak_send(&t, channel);
        if (err == ESP_OK) {
            msg_t *m = msg_add();
            m->outgoing = true;
            m->from = s_me;
            m->to = MT_BROADCAST;
            m->channel = channel;
            edge_strlcpy(m->status, "sent", sizeof(m->status));
            snprintf(m->text, sizeof(m->text), "[TAK] %s: %s", t.callsign, text);
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t mesh_tak_send_position(uint8_t channel)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = running_ok();
    if (err == ESP_OK && (channel >= s_nch || !tak_send_pli(channel))) err = ESP_ERR_NOT_FOUND;
    xSemaphoreGive(s_lock);
    return err;
}

cJSON *mesh_tak_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; s_tak && i < MAX_TAK; i++) {
        const tak_t *x = &s_tak[i];
        if (!x->used) continue;
        cJSON *o = cJSON_CreateObject();
        char id[12];
        node_id_str(x->node, id);
        cJSON_AddStringToObject(o, "node", id);
        char c[sizeof(x->callsign)];
        edge_sanitize_text(c, sizeof(c), (const uint8_t *)x->callsign, strlen(x->callsign));
        cJSON_AddStringToObject(o, "callsign", c);
        edge_sanitize_text(c, sizeof(c), (const uint8_t *)x->uid, strlen(x->uid) < sizeof(c) ? strlen(x->uid) : sizeof(c) - 1);
        cJSON_AddStringToObject(o, "uid", c);
        cJSON_AddStringToObject(o, "team", TAK_TEAMS[x->team < 15 ? x->team : 0]);
        cJSON_AddStringToObject(o, "role", TAK_ROLES[x->role < 9 ? x->role : 0]);
        if (x->battery) cJSON_AddNumberToObject(o, "battery", x->battery);
        cJSON_AddStringToObject(o, "plugin", x->v2 ? "v2" : "v1");
        cJSON_AddBoolToObject(o, "compressed", x->compressed);
        if (x->has_pos) {
            cJSON *p = cJSON_AddObjectToObject(o, "reported_position");
            cJSON_AddNumberToObject(p, "lat", x->lat_i / 1e7);
            cJSON_AddNumberToObject(p, "lon", x->lon_i / 1e7);
            cJSON_AddNumberToObject(p, "alt_m", x->alt);
            cJSON_AddNumberToObject(p, "speed_mps", x->speed);
            cJSON_AddNumberToObject(p, "course_deg", x->course / 100.0);
            cJSON_AddStringToObject(p, "provenance", "NETWORK");
        }
        cJSON_AddNumberToObject(o, "ago_s", (double)((now_ms() - x->at) / 1000));
        if (x->at_s) cJSON_AddNumberToObject(o, "unix_s", x->at_s);
        cJSON_AddItemToArray(a, o);
    }
    xSemaphoreGive(s_lock);
    return a;
}
