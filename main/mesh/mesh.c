#include "mesh.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aes/esp_aes.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "edge_config.h"
#include "edge_events.h"
#include "edge_log.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"
#include "mesh_pb.h"
#include "mesh_radio.h"
#include "sx1262.h"

static const char *TAG = "mesh";

#define TXQ_LEN         16
#define SEEN_LEN        128
#define SEEN_TTL_MS     (10 * 60 * 1000)
#define MAX_NODES       64
#define MAX_MSGS        16
#define MAX_PENDING     8
#define MAX_TEXT        200
#define ACK_RETRIES     3

#define REQ_START   0x01
#define REQ_STOP    0x02
#define REQ_RELOAD  0x04

typedef struct {
    bool used;
    bool is_relay;
    bool track;          /* our own packet: report status, maybe wait for ACK */
    uint8_t len;
    uint8_t tries;
    uint32_t from;
    uint32_t id;
    uint32_t to;
    int64_t not_before;
    uint8_t buf[MESH_MAX_PACKET];
} tx_item_t;

typedef struct {
    bool used;
    bool broadcast;
    uint32_t id;
    uint32_t to;
    int64_t deadline;
    uint8_t tries;
    uint8_t len;
    uint8_t buf[MESH_MAX_PACKET];
} pending_t;

typedef struct {
    uint32_t from;
    uint32_t id;
    int64_t at;
} seen_t;

typedef struct {
    uint32_t num;
    bool has_user;
    char long_name[40];
    char short_name[8];
    uint32_t hw_model;
    bool has_pkc;
    int64_t last_heard;
    int64_t last_nodeinfo_reply;
    int16_t rssi;
    float snr;
    int8_t hops_away;    /* -1 unknown */
    bool has_pos;
    int32_t lat_i;
    int32_t lon_i;
    int32_t alt;
    uint32_t pos_time;
} node_t;

typedef struct {
    bool used;
    bool outgoing;
    uint32_t id;
    uint32_t from;
    uint32_t to;
    uint8_t channel;
    int64_t at;
    int16_t rssi;
    float snr;
    char status[12];
    char text[MAX_TEXT + 34];
} msg_t;

typedef struct {
    char name[12];       /* effective name used for hashing */
    uint8_t key[32];
    size_t key_len;
    uint8_t hash;
} chan_t;

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static volatile uint32_t s_req;
static bool s_radio_ok;
static bool s_running;

static edge_config_t s_cfg;
static const mesh_region_t *s_region;
static const mesh_preset_t *s_preset;
static float s_freq;
static chan_t s_ch[EDGE_MESH_MAX_CHANNELS];
static int s_nch;
static uint32_t s_me;

static tx_item_t *s_txq;      /* allocated only while mesh mode runs */
static bool s_tx_busy;
static tx_item_t s_tx_cur;
static int64_t s_tx_started;
static uint32_t s_tx_airtime;
static pending_t *s_pending;
static seen_t s_seen[SEEN_LEN];
static int s_seen_next;
static node_t s_nodes[MAX_NODES];
static msg_t s_msgs[MAX_MSGS];
static int s_msg_next;
static int64_t s_rx_activity;
static int64_t s_next_nodeinfo;
static uint32_t s_air_min[60];  /* airtime per minute bucket, ms */
static int64_t s_air_bucket_start;
static int s_air_idx;

static struct {
    uint32_t rx_ok, rx_crc, rx_undecoded, rx_dupe, tx_ok, relayed, relay_cancelled, tx_dropped;
} s_st;

/* ---- Small helpers ---------------------------------------------------------- */

static int64_t now_ms(void)
{
    return edge_uptime_ms();
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

static void crypt(const chan_t *ch, uint32_t from, uint32_t id, uint8_t *data, size_t n)
{
    if (ch->key_len == 0 || n == 0) return;
    uint8_t nonce[16] = {0};
    uint64_t pid = id;
    memcpy(nonce, &pid, 8);       /* packet id, little-endian 64-bit */
    memcpy(nonce + 8, &from, 4);  /* sender node number, little-endian */
    uint8_t stream[16];
    size_t off = 0;
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    esp_aes_setkey(&ctx, ch->key, ch->key_len * 8);
    esp_aes_crypt_ctr(&ctx, n, &off, nonce, stream, data, data);
    esp_aes_free(&ctx);
}

static bool seen_check_add(uint32_t from, uint32_t id)
{
    int64_t t = now_ms();
    for (int i = 0; i < SEEN_LEN; i++) {
        if (s_seen[i].at && s_seen[i].from == from && s_seen[i].id == id && t - s_seen[i].at < SEEN_TTL_MS) {
            return true;
        }
    }
    s_seen[s_seen_next] = (seen_t){.from = from, .id = id, .at = t};
    s_seen_next = (s_seen_next + 1) % SEEN_LEN;
    return false;
}

static node_t *node_get(uint32_t num, bool create)
{
    node_t *oldest = NULL;
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].num == num) return &s_nodes[i];
    }
    if (!create) return NULL;
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].num == 0) {
            oldest = &s_nodes[i];
            break;
        }
        if (!oldest || s_nodes[i].last_heard < oldest->last_heard) oldest = &s_nodes[i];
    }
    memset(oldest, 0, sizeof(*oldest));
    oldest->num = num;
    oldest->hops_away = -1;
    return oldest;
}

static int node_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_NODES; i++) n += s_nodes[i].num != 0;
    return n;
}

static cJSON *node_json(const node_t *n)
{
    cJSON *o = cJSON_CreateObject();
    char id[12];
    node_id_str(n->num, id);
    cJSON_AddStringToObject(o, "id", id);
    cJSON_AddNumberToObject(o, "num", n->num);
    if (n->has_user) {
        char s[sizeof(n->long_name)];
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)n->long_name, strlen(n->long_name));
        cJSON_AddStringToObject(o, "long_name", s);
        edge_sanitize_text(s, sizeof(s), (const uint8_t *)n->short_name, strlen(n->short_name));
        cJSON_AddStringToObject(o, "short_name", s);
        cJSON_AddNumberToObject(o, "hw_model", n->hw_model);
        cJSON_AddBoolToObject(o, "has_public_key", n->has_pkc);
    }
    cJSON_AddNumberToObject(o, "last_heard_ago_s", (double)((now_ms() - n->last_heard) / 1000));
    int64_t u = edge_time_to_unix_ms(n->last_heard);
    if (u > 0) cJSON_AddNumberToObject(o, "last_heard_unix_ms", (double)u);
    cJSON_AddNumberToObject(o, "rssi", n->rssi);
    cJSON_AddNumberToObject(o, "snr", n->snr);
    if (n->hops_away >= 0) cJSON_AddNumberToObject(o, "hops_away", n->hops_away);
    if (n->has_pos) {
        /* Self-reported by that node over the mesh, not measured by the Edge. */
        cJSON *p = cJSON_AddObjectToObject(o, "reported_position");
        cJSON_AddNumberToObject(p, "lat", n->lat_i / 1e7);
        cJSON_AddNumberToObject(p, "lon", n->lon_i / 1e7);
        cJSON_AddNumberToObject(p, "alt_m", n->alt);
        if (n->pos_time) cJSON_AddNumberToObject(p, "reported_unix_s", n->pos_time);
        cJSON_AddStringToObject(p, "provenance", "NETWORK");
    }
    return o;
}

static msg_t *msg_add(void)
{
    msg_t *m = &s_msgs[s_msg_next];
    s_msg_next = (s_msg_next + 1) % MAX_MSGS;
    memset(m, 0, sizeof(*m));
    m->used = true;
    m->at = now_ms();
    return m;
}

static msg_t *msg_find_out(uint32_t id)
{
    for (int i = 0; i < MAX_MSGS; i++) {
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
    cJSON_AddNumberToObject(o, "ago_s", (double)((now_ms() - m->at) / 1000));
    int64_t u = edge_time_to_unix_ms(m->at);
    if (u > 0) cJSON_AddNumberToObject(o, "unix_ms", (double)u);
    if (m->outgoing) {
        cJSON_AddStringToObject(o, "status", m->status);
    } else {
        cJSON_AddNumberToObject(o, "rssi", m->rssi);
        cJSON_AddNumberToObject(o, "snr", m->snr);
    }
    cJSON_AddStringToObject(o, "text", m->text);
    node_t *n = node_get(m->outgoing ? m->to : m->from, false);
    if (n && n->has_user) cJSON_AddStringToObject(o, m->outgoing ? "to_name" : "from_name", n->short_name);
    return o;
}

static void publish_tx_status(uint32_t id, const char *status, int reason)
{
    msg_t *m = msg_find_out(id);
    if (m) edge_strlcpy(m->status, status, sizeof(m->status));
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_tx");
    cJSON_AddNumberToObject(ev, "id", id);
    cJSON_AddStringToObject(ev, "status", status);
    if (reason >= 0) cJSON_AddNumberToObject(ev, "reason", reason);
    edge_events_publish(ev);
    cJSON_Delete(ev);
}

static void log_text(uint32_t from, uint32_t to, uint32_t id, uint8_t ch, uint8_t flags_hops, int16_t rssi,
                     float snr, bool outgoing, const char *text, size_t len)
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
    b->hops = flags_hops;
    b->rssi = (int8_t)(rssi < -128 ? -128 : rssi > 127 ? 127 : rssi);
    b->snr_x4 = (int8_t)(snr * 4);
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

/* ---- Airtime / duty cycle --------------------------------------------------- */

static void air_roll(void)
{
    int64_t t = now_ms();
    if (s_air_bucket_start == 0) s_air_bucket_start = t;
    while (t - s_air_bucket_start >= 60000) {
        s_air_bucket_start += 60000;
        s_air_idx = (s_air_idx + 1) % 60;
        s_air_min[s_air_idx] = 0;
    }
}

static uint32_t air_last_hour(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 60; i++) sum += s_air_min[i];
    return sum;
}

static bool duty_allows(uint32_t airtime)
{
    if (!s_region || s_region->duty_cycle >= 100) return true;
    air_roll();
    uint32_t budget = (uint32_t)s_region->duty_cycle * 36000u; /* ms per hour */
    return air_last_hour() + airtime <= budget;
}

/* ---- Transmit queue --------------------------------------------------------- */

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
        if (s_txq[i].used && s_txq[i].is_relay && s_txq[i].from == from && s_txq[i].id == id) {
            s_txq[i].used = false;
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

/* Encrypts and queues a packet originated by this node. Caller holds s_lock. */
static esp_err_t send_data(uint32_t to, int ch_idx, const mt_data_t *d, bool want_ack, bool track,
                           uint32_t *out_id)
{
    if (!s_running || !s_region) return ESP_ERR_INVALID_STATE;
    if (ch_idx < 0 || ch_idx >= s_nch) return ESP_ERR_INVALID_ARG;
    tx_item_t *t = txq_alloc();
    if (!t) return ESP_ERR_NO_MEM;

    uint8_t plain[MESH_MAX_PAYLOAD];
    size_t n = mt_data_encode(d, plain, sizeof(plain));
    if (n == 0) {
        t->used = false;
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t id = new_packet_id();
    mt_header_t h = {
        .to = to,
        .from = s_me,
        .id = id,
        .flags = mt_make_flags(s_cfg.hop_limit, s_cfg.hop_limit, want_ack),
        .channel = s_ch[ch_idx].hash,
        .next_hop = 0,
        .relay_node = (uint8_t)(s_me & 0xff),
    };
    mt_header_write(&h, t->buf);
    memcpy(t->buf + MESH_HEADER_LEN, plain, n);
    crypt(&s_ch[ch_idx], s_me, id, t->buf + MESH_HEADER_LEN, n);
    t->len = (uint8_t)(MESH_HEADER_LEN + n);
    t->from = s_me;
    t->id = id;
    t->to = to;
    t->track = track;
    t->tries = 0;
    t->not_before = now_ms() + rnd(8) * mesh_slot_ms(s_preset);
    seen_check_add(s_me, id);
    if (out_id) *out_id = id;
    if (s_task) xTaskNotifyGive(s_task);
    return ESP_OK;
}

static void send_ack(uint32_t to, uint32_t req_id, int ch_idx)
{
    uint8_t rp[8];
    size_t rl = mt_routing_encode_error(0, rp, sizeof(rp));
    mt_data_t d = {.portnum = MT_PORT_ROUTING, .payload = rp, .payload_len = rl, .request_id = req_id};
    send_data(to, ch_idx, &d, false, false, NULL);
}

static void send_nodeinfo(uint32_t to, bool want_response)
{
    mt_user_t u = {.hw_model = MT_HW_HELTEC_V3, .role = MT_ROLE_CLIENT};
    snprintf(u.id, sizeof(u.id), "!%08lx", (unsigned long)s_me);
    edge_strlcpy(u.long_name, s_cfg.long_name, sizeof(u.long_name));
    edge_strlcpy(u.short_name, s_cfg.short_name, sizeof(u.short_name));
    uint8_t ub[96];
    size_t ul = mt_user_encode(&u, ub, sizeof(ub));
    mt_data_t d = {.portnum = MT_PORT_NODEINFO, .payload = ub, .payload_len = ul,
                   .want_response = want_response, .has_bitfield = true, .bitfield = 0};
    send_data(to, 0, &d, false, false, NULL);
}

/* ---- Receive ---------------------------------------------------------------- */

static void handle_text(const mt_header_t *h, int ch, const mt_data_t *d, int16_t rssi, float snr)
{
    if (h->to != s_me && h->to != MT_BROADCAST) return; /* someone else's direct message */
    msg_t *m = msg_add();
    m->outgoing = false;
    m->id = h->id;
    m->from = h->from;
    m->to = h->to;
    m->channel = (uint8_t)ch;
    m->rssi = rssi;
    m->snr = snr;
    edge_sanitize_text(m->text, sizeof(m->text), d->payload, d->payload_len);

    log_text(h->from, h->to, h->id, (uint8_t)ch, (uint8_t)(mt_hop_limit(h) | (mt_hop_start(h) << 4)), rssi, snr,
             false, (const char *)d->payload, d->payload_len);

    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "mesh_rx");
    cJSON_AddItemToObject(ev, "msg", msg_json(m));
    edge_events_publish(ev);
    cJSON_Delete(ev);
    ESP_LOGI(TAG, "text from !%08lx on ch %d (%d dBm)", (unsigned long)h->from, ch, rssi);
}

static void handle_data(const mt_header_t *h, int ch, const mt_data_t *d, int16_t rssi, float snr)
{
    node_t *n = node_get(h->from, true);
    switch (d->portnum) {
    case MT_PORT_TEXT:
        if (d->payload_len) handle_text(h, ch, d, rssi, snr);
        break;
    case MT_PORT_NODEINFO: {
        mt_user_t u;
        if (mt_user_decode(d->payload, d->payload_len, &u)) {
            n->has_user = true;
            edge_strlcpy(n->long_name, u.long_name, sizeof(n->long_name));
            edge_strlcpy(n->short_name, u.short_name, sizeof(n->short_name));
            n->hw_model = u.hw_model;
            n->has_pkc = u.has_public_key;
            cJSON *ev = cJSON_CreateObject();
            cJSON_AddStringToObject(ev, "ev", "mesh_node");
            cJSON_AddItemToObject(ev, "node", node_json(n));
            edge_events_publish(ev);
            cJSON_Delete(ev);
        }
        bool for_us = h->to == s_me || h->to == MT_BROADCAST;
        if (d->want_response && for_us && now_ms() - n->last_nodeinfo_reply > 10 * 60 * 1000) {
            n->last_nodeinfo_reply = now_ms();
            send_nodeinfo(h->from, false);
        }
        break;
    }
    case MT_PORT_POSITION: {
        mt_position_t p;
        if (mt_position_decode(d->payload, d->payload_len, &p) && p.has_lat && p.has_lon) {
            n->has_pos = true;
            n->lat_i = p.lat_i;
            n->lon_i = p.lon_i;
            n->alt = p.has_alt ? p.alt : 0;
            n->pos_time = p.time;
        }
        break;
    }
    case MT_PORT_ROUTING: {
        if (h->to != s_me || !d->request_id) break;
        mt_routing_t r;
        if (!mt_routing_decode(d->payload, d->payload_len, &r)) break;
        for (int i = 0; i < MAX_PENDING; i++) {
            if (s_pending[i].used && s_pending[i].id == d->request_id) {
                s_pending[i].used = false;
                if (!r.has_error || r.error_reason == 0) {
                    publish_tx_status(d->request_id, "delivered", -1);
                } else {
                    publish_tx_status(d->request_id, "failed", (int)r.error_reason);
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

static void process_rx(uint8_t *buf, size_t len, int16_t rssi, float snr)
{
    if (len < MESH_HEADER_LEN) return;
    mt_header_t h;
    mt_header_read(buf, &h);
    uint8_t *payload = buf + MESH_HEADER_LEN;
    size_t plen = len - MESH_HEADER_LEN;

    if (h.from == s_me) {
        /* Our own packet relayed by a neighbour: it reached at least one node. */
        for (int i = 0; i < MAX_PENDING; i++) {
            if (s_pending[i].used && s_pending[i].broadcast && s_pending[i].id == h.id) {
                s_pending[i].used = false;
                publish_tx_status(h.id, "relayed", -1);
            }
        }
        return;
    }
    if (seen_check_add(h.from, h.id)) {
        s_st.rx_dupe++;
        if (txq_cancel_relay(h.from, h.id)) s_st.relay_cancelled++; /* someone else relayed it first */
        return;
    }
    s_st.rx_ok++;

    node_t *n = node_get(h.from, true);
    n->last_heard = now_ms();
    n->rssi = rssi;
    n->snr = snr;
    if (mt_hop_start(&h) >= mt_hop_limit(&h) && mt_hop_start(&h) > 0) {
        n->hops_away = (int8_t)(mt_hop_start(&h) - mt_hop_limit(&h));
    }

    bool decoded = false;
    int ch_idx = -1;
    mt_data_t d;
    uint8_t tmp[MESH_MAX_PAYLOAD];
    for (int i = 0; i < s_nch && !decoded; i++) {
        if (s_ch[i].hash != h.channel) continue;
        memcpy(tmp, payload, plen);
        crypt(&s_ch[i], h.from, h.id, tmp, plen);
        if (mt_data_decode(tmp, plen, &d)) {
            decoded = true;
            ch_idx = i;
        }
    }
    if (decoded) {
        handle_data(&h, ch_idx, &d, rssi, snr);
        if (h.to == s_me && mt_want_ack(&h)) send_ack(h.from, h.id, ch_idx);
    } else {
        s_st.rx_undecoded++;
    }

    /* Managed flooding: relay what is not addressed to us while hops remain. */
    bool relay = h.to != s_me && mt_hop_limit(&h) > 0 &&
                 (s_cfg.rebroadcast == EDGE_REBROADCAST_ALL ||
                  (s_cfg.rebroadcast == EDGE_REBROADCAST_LOCAL_ONLY && decoded));
    if (relay) {
        tx_item_t *t = txq_alloc();
        if (t) {
            memcpy(t->buf, buf, len);
            t->buf[12] = (uint8_t)((h.flags & ~MT_FLAG_HOP_LIMIT_MASK) | ((mt_hop_limit(&h) - 1) & MT_FLAG_HOP_LIMIT_MASK));
            t->buf[15] = (uint8_t)(s_me & 0xff); /* relay_node */
            t->len = (uint8_t)len;
            t->is_relay = true;
            t->from = h.from;
            t->id = h.id;
            t->to = h.to;
            /* SNR-weighted contention window: weaker links relay first. */
            uint32_t slot = mesh_slot_ms(s_preset);
            float s = snr < -20 ? -20 : snr > 10 ? 10 : snr;
            uint32_t cw = 3 + (uint32_t)((s + 20) * 5 / 30); /* 3..8 */
            t->not_before = now_ms() + 2 * 8 * slot + rnd(1u << cw) * slot;
        }
    }
}

/* ---- Radio control ---------------------------------------------------------- */

static void apply_config(void)
{
    edge_config_get(&s_cfg);
    s_me = s_cfg.node_num;
    s_region = mesh_region_by_code(s_cfg.region);
    s_preset = mesh_preset_by_code(s_cfg.preset);
    if (!s_preset) s_preset = mesh_preset_by_code(0);
    s_nch = s_cfg.n_channels;
    for (int i = 0; i < s_nch; i++) {
        const edge_mesh_channel_t *c = &s_cfg.channels[i];
        edge_strlcpy(s_ch[i].name, mesh_effective_channel_name(c->name, s_preset), sizeof(s_ch[i].name));
        s_ch[i].key_len = mesh_expand_psk(c->psk, c->psk_len, s_ch[i].key);
        s_ch[i].hash = mesh_channel_hash(s_ch[i].name, s_ch[i].key, s_ch[i].key_len);
    }
    if (!s_region) {
        s_freq = 0;
        ESP_LOGW(TAG, "LoRa region not set: radio stays silent");
        return;
    }
    s_freq = mesh_frequency_mhz(s_region, s_preset, s_cfg.channels[0].name, s_cfg.channel_num);
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
    };
    if (sx1262_configure(&rc) == ESP_OK) sx1262_start_rx();
    ESP_LOGI(TAG, "node !%08lx on %s %s, %.3f MHz, ch0 hash 0x%02x", (unsigned long)s_me, s_region->name,
             s_preset->display, s_freq, s_ch[0].hash);
}

static void do_start(void)
{
    if (!s_radio_ok) return;
    if (!s_txq) s_txq = calloc(TXQ_LEN, sizeof(tx_item_t));
    if (!s_pending) s_pending = calloc(MAX_PENDING, sizeof(pending_t));
    if (!s_txq || !s_pending) {
        ESP_LOGE(TAG, "not enough memory for the mesh queues");
        return;
    }
    if (s_txq) memset(s_txq, 0, TXQ_LEN * sizeof(tx_item_t));
    if (s_pending) memset(s_pending, 0, MAX_PENDING * sizeof(pending_t));
    s_tx_busy = false;
    s_running = true;
    apply_config();
    s_next_nodeinfo = now_ms() + 5000 + rnd(10000);
}

static void do_stop(void)
{
    s_running = false;
    s_tx_busy = false;
    if (s_txq) memset(s_txq, 0, TXQ_LEN * sizeof(tx_item_t));
    if (s_radio_ok) sx1262_sleep();
    free(s_txq);
    free(s_pending);
    s_txq = NULL;
    s_pending = NULL;
}

static bool channel_busy(void)
{
    if (!s_rx_activity) return false;
    if (now_ms() - s_rx_activity > (int64_t)mesh_airtime_ms(s_preset, MESH_MAX_PACKET)) {
        s_rx_activity = 0; /* stale preamble detection */
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
        if (irq & (SX_IRQ_CRC_ERR | SX_IRQ_HEADER_ERR)) {
            s_rx_activity = 0;
            s_st.rx_crc++;
        } else if (irq & SX_IRQ_RX_DONE) {
            s_rx_activity = 0;
            uint8_t buf[MESH_MAX_PACKET];
            size_t len = 0;
            int16_t rssi = 0;
            float snr = 0;
            if (sx1262_read_packet(buf, sizeof(buf), &len, &rssi, &snr) == ESP_OK) {
                process_rx(buf, len, rssi, snr);
            }
        }
        if (irq & SX_IRQ_TX_DONE && s_tx_busy) {
            s_tx_busy = false;
            s_st.tx_ok++;
            air_roll();
            s_air_min[s_air_idx] += s_tx_airtime;
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
        if (s_txq[i].used && s_txq[i].not_before <= t && (!best || s_txq[i].not_before < best->not_before)) {
            best = &s_txq[i];
        }
    }
    if (!best) return;
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

static uint32_t next_wait_ms(void)
{
    int64_t t = now_ms();
    int64_t wait = 100; /* also polls DIO1 in case an edge was missed */
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

    for (;;) {
        uint32_t wait = s_running ? next_wait_ms() : 1000;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));

        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t req = __atomic_exchange_n(&s_req, 0, __ATOMIC_SEQ_CST);
        if (req & REQ_STOP) do_stop();
        if (req & REQ_START) do_start();
        if ((req & REQ_RELOAD) && s_running) {
            if (s_txq) memset(s_txq, 0, TXQ_LEN * sizeof(tx_item_t));
            s_tx_busy = false;
            apply_config();
        }
        if (s_running && s_region) {
            handle_irq();
            if (sx1262_dio1_high()) handle_irq();
            service_pending();
            service_tx();
            if (s_cfg.nodeinfo_interval_s && now_ms() >= s_next_nodeinfo) {
                send_nodeinfo(MT_BROADCAST, false);
                s_next_nodeinfo = now_ms() + (int64_t)s_cfg.nodeinfo_interval_s * 1000;
            }
        }
        xSemaphoreGive(s_lock);
    }
}

/* ---- Public API ------------------------------------------------------------- */

esp_err_t mesh_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    edge_config_get(&s_cfg);
    s_me = s_cfg.node_num;
    if (xTaskCreatePinnedToCore(mesh_task, "mesh", 6144, NULL, 6, &s_task, 1) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static void request(uint32_t bit)
{
    __atomic_or_fetch(&s_req, bit, __ATOMIC_SEQ_CST);
    if (s_task) xTaskNotifyGive(s_task);
}

void mesh_start(void)
{
    request(REQ_START);
}

void mesh_stop(void)
{
    request(REQ_STOP);
}

void mesh_reload(void)
{
    request(REQ_RELOAD);
}

bool mesh_running(void)
{
    return s_running;
}

esp_err_t mesh_send_text(uint32_t to, uint8_t channel, const char *text, bool want_ack, uint32_t *out_id)
{
    if (!text) return ESP_ERR_INVALID_ARG;
    size_t len = strlen(text);
    if (len == 0 || len > MAX_TEXT) return ESP_ERR_INVALID_SIZE;
    if (to == 0) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)text, .payload_len = len,
                   .has_bitfield = true, .bitfield = 0};
    uint32_t id = 0;
    bool ack = want_ack && to != MT_BROADCAST;
    esp_err_t err = send_data(to, channel, &d, ack, true, &id);
    if (err == ESP_OK) {
        msg_t *m = msg_add();
        m->outgoing = true;
        m->id = id;
        m->from = s_me;
        m->to = to;
        m->channel = channel;
        edge_strlcpy(m->status, "queued", sizeof(m->status));
        edge_sanitize_text(m->text, sizeof(m->text), (const uint8_t *)text, len);
        log_text(s_me, to, id, channel, s_cfg.hop_limit, 0, 0, true, text, len);
        if (out_id) *out_id = id;
    }
    xSemaphoreGive(s_lock);
    return err;
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
    int last = (s_msg_next + MAX_MSGS - 1) % MAX_MSGS;
    for (int k = 0; k < MAX_MSGS; k++) {
        msg_t *m = &s_msgs[(last + MAX_MSGS - k) % MAX_MSGS];
        if (m->used && !m->outgoing) {
            node_t *n = node_get(m->from, false);
            if (n && n->has_user && n->short_name[0]) {
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
    if (s_region) {
        cJSON_AddStringToObject(o, "region", s_region->name);
        cJSON_AddNumberToObject(o, "freq_mhz", s_freq);
        cJSON_AddNumberToObject(o, "duty_cycle_pct", s_region->duty_cycle);
        air_roll();
        cJSON_AddNumberToObject(o, "airtime_last_hour_ms", air_last_hour());
    } else {
        cJSON_AddNullToObject(o, "region");
    }
    if (s_preset) cJSON_AddStringToObject(o, "preset", s_preset->name);
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
    cJSON *st = cJSON_AddObjectToObject(o, "counters");
    cJSON_AddNumberToObject(st, "rx", s_st.rx_ok);
    cJSON_AddNumberToObject(st, "rx_bad", s_st.rx_crc);
    cJSON_AddNumberToObject(st, "rx_other_channel", s_st.rx_undecoded);
    cJSON_AddNumberToObject(st, "rx_duplicate", s_st.rx_dupe);
    cJSON_AddNumberToObject(st, "tx", s_st.tx_ok);
    cJSON_AddNumberToObject(st, "relayed", s_st.relayed);
    cJSON_AddNumberToObject(st, "relay_skipped", s_st.relay_cancelled);
    cJSON_AddNumberToObject(st, "tx_dropped", s_st.tx_dropped);
    xSemaphoreGive(s_lock);
    return o;
}

cJSON *mesh_nodes_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].num) cJSON_AddItemToArray(a, node_json(&s_nodes[i]));
    }
    xSemaphoreGive(s_lock);
    return a;
}

cJSON *mesh_messages_json(void)
{
    cJSON *a = cJSON_CreateArray();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int k = 0; k < MAX_MSGS; k++) {
        msg_t *m = &s_msgs[(s_msg_next + k) % MAX_MSGS]; /* oldest first */
        if (m->used) cJSON_AddItemToArray(a, msg_json(m));
    }
    xSemaphoreGive(s_lock);
    return a;
}
