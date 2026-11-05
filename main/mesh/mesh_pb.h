// Minimal protobuf encoder/decoder for the handful of Meshtastic messages the
// Edge understands (Data, User, Position, Routing) plus the 16-byte on-air
// packet header. Pure C, host-tested.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PortNum values used here. */
#define MT_PORT_UNKNOWN     0
#define MT_PORT_TEXT        1
#define MT_PORT_POSITION    3
#define MT_PORT_NODEINFO    4
#define MT_PORT_ROUTING     5
#define MT_PORT_REMOTE_HW   2
#define MT_PORT_ADMIN       6
#define MT_PORT_WAYPOINT    8
#define MT_PORT_DETECTION   10
#define MT_PORT_PAXCOUNTER  34
#define MT_PORT_SERIAL      64
#define MT_PORT_STORE_FWD   65
#define MT_PORT_RANGE_TEST  66
#define MT_PORT_TELEMETRY   67
#define MT_PORT_TRACEROUTE  70
#define MT_PORT_NEIGHBORINFO 71

#define MT_HW_HELTEC_V3     43
#define MT_ROLE_CLIENT      0
#define MT_BROADCAST        0xffffffffu

/* Header flag bits. */
#define MT_FLAG_HOP_LIMIT_MASK  0x07
#define MT_FLAG_WANT_ACK        0x08
#define MT_FLAG_VIA_MQTT        0x10
#define MT_FLAG_HOP_START_SHIFT 5
#define MT_FLAG_HOP_START_MASK  0xe0

typedef struct {
    uint32_t to;
    uint32_t from;
    uint32_t id;
    uint8_t flags;
    uint8_t channel;    /* channel hash */
    uint8_t next_hop;
    uint8_t relay_node;
} mt_header_t;

void mt_header_write(const mt_header_t *h, uint8_t out[16]);
void mt_header_read(const uint8_t in[16], mt_header_t *h);

static inline uint8_t mt_hop_limit(const mt_header_t *h) { return h->flags & MT_FLAG_HOP_LIMIT_MASK; }
static inline uint8_t mt_hop_start(const mt_header_t *h) { return (h->flags & MT_FLAG_HOP_START_MASK) >> MT_FLAG_HOP_START_SHIFT; }
static inline bool mt_want_ack(const mt_header_t *h) { return (h->flags & MT_FLAG_WANT_ACK) != 0; }
uint8_t mt_make_flags(uint8_t hop_limit, uint8_t hop_start, bool want_ack);

/* ---- Data ---------------------------------------------------------------- */
typedef struct {
    uint32_t portnum;
    const uint8_t *payload;
    size_t payload_len;
    bool want_response;
    uint32_t dest;
    uint32_t source;
    uint32_t request_id;
    uint32_t reply_id;
    uint32_t emoji;
    bool has_bitfield;
    uint32_t bitfield;
} mt_data_t;

bool mt_data_decode(const uint8_t *buf, size_t n, mt_data_t *out);
/** Returns encoded length, 0 on overflow. */
size_t mt_data_encode(const mt_data_t *d, uint8_t *buf, size_t cap);

/* ---- User (NODEINFO_APP payload) ----------------------------------------- */
typedef struct {
    char id[16];
    char long_name[40];
    char short_name[8];
    uint32_t hw_model;
    uint32_t role;
    bool is_licensed;
    bool has_public_key;
    uint8_t public_key[32];
} mt_user_t;

bool mt_user_decode(const uint8_t *buf, size_t n, mt_user_t *out);
size_t mt_user_encode(const mt_user_t *u, uint8_t *buf, size_t cap);

/* ---- Position (POSITION_APP payload) ------------------------------------- */
typedef struct {
    bool has_lat;
    bool has_lon;
    bool has_alt;
    int32_t lat_i;      /* 1e-7 degrees */
    int32_t lon_i;
    int32_t alt;        /* metres */
    uint32_t time;      /* unix seconds as reported by the sender */
    uint32_t location_source; /* 1 = manual */
    uint32_t precision_bits;  /* 0 = not given, 32 = full */
} mt_position_t;

bool mt_position_decode(const uint8_t *buf, size_t n, mt_position_t *out);
size_t mt_position_encode(const mt_position_t *p, uint8_t *buf, size_t cap);
/** Meshtastic position precision: keep the top `bits` of each coordinate, centred in the cell. */
int32_t mt_position_truncate(int32_t coord_i, uint32_t bits);

/* ---- Telemetry (TELEMETRY_APP) -------------------------------------------- */
typedef struct {
    uint32_t time;
    bool has_device;
    bool has_battery, has_voltage, has_ch_util, has_air_util, has_uptime;
    uint32_t battery_level;     /* 0-100, 101 = external power */
    float voltage;
    float channel_utilization;  /* percent */
    float air_util_tx;          /* percent */
    uint32_t uptime_seconds;
    bool has_env;
    bool has_temp, has_hum, has_press;
    float temperature, relative_humidity, barometric_pressure;
} mt_telemetry_t;

bool mt_telemetry_decode(const uint8_t *buf, size_t n, mt_telemetry_t *out);
size_t mt_telemetry_encode(const mt_telemetry_t *t, uint8_t *buf, size_t cap);

/* ---- RouteDiscovery (TRACEROUTE_APP) --------------------------------------- */
#define MT_MAX_ROUTE 8
typedef struct {
    uint8_t n_route, n_snr_towards, n_route_back, n_snr_back;
    uint32_t route[MT_MAX_ROUTE];
    int8_t snr_towards[MT_MAX_ROUTE + 1];   /* SNR x 4 */
    uint32_t route_back[MT_MAX_ROUTE];
    int8_t snr_back[MT_MAX_ROUTE + 1];
} mt_route_t;

bool mt_route_decode(const uint8_t *buf, size_t n, mt_route_t *out);
size_t mt_route_encode(const mt_route_t *r, uint8_t *buf, size_t cap);

/* ---- NeighborInfo (NEIGHBORINFO_APP) --------------------------------------- */
#define MT_MAX_NEIGHBORS 10
typedef struct {
    uint32_t node_id;
    float snr;
    uint32_t last_rx_time;
    uint32_t interval_secs;
} mt_neighbor_t;

typedef struct {
    uint32_t node_id;
    uint32_t last_sent_by_id;
    uint32_t interval_secs;
    uint8_t n;
    mt_neighbor_t nb[MT_MAX_NEIGHBORS];
} mt_neighborinfo_t;

bool mt_neighborinfo_decode(const uint8_t *buf, size_t n, mt_neighborinfo_t *out);
size_t mt_neighborinfo_encode(const mt_neighborinfo_t *ni, uint8_t *buf, size_t cap);

/* ---- Waypoint (WAYPOINT_APP) ----------------------------------------------- */
typedef struct {
    uint32_t id;
    bool has_lat, has_lon;
    int32_t lat_i, lon_i;
    uint32_t expire;        /* unix seconds, 0 = never */
    uint32_t locked_to;
    char name[32];
    char description[100];
    uint32_t icon;          /* unicode code point */
} mt_waypoint_t;

bool mt_waypoint_decode(const uint8_t *buf, size_t n, mt_waypoint_t *out);
size_t mt_waypoint_encode(const mt_waypoint_t *w, uint8_t *buf, size_t cap);

/* ---- Paxcount (PAXCOUNTER_APP) ---------------------------------------------- */
size_t mt_paxcount_encode(uint32_t wifi, uint32_t ble, uint32_t uptime, uint8_t *buf, size_t cap);
bool mt_paxcount_decode(const uint8_t *buf, size_t n, uint32_t *wifi, uint32_t *ble, uint32_t *uptime);

/* ---- StoreAndForward (STORE_FORWARD_APP) ------------------------------------ */
#define MT_SF_ROUTER_HEARTBEAT      2
#define MT_SF_ROUTER_HISTORY        6
#define MT_SF_ROUTER_TEXT_DIRECT    8
#define MT_SF_ROUTER_TEXT_BROADCAST 9
#define MT_SF_CLIENT_HISTORY        65

typedef struct {
    uint32_t rr;
    bool has_heartbeat;
    uint32_t hb_period, hb_secondary;
    bool has_history;
    uint32_t history_messages, window, last_request;
    const uint8_t *text;
    size_t text_len;
} mt_sf_t;

bool mt_sf_decode(const uint8_t *buf, size_t n, mt_sf_t *out);
size_t mt_sf_encode(const mt_sf_t *sf, uint8_t *buf, size_t cap);

/* ---- ChannelSet (the https://meshtastic.org/e/#... link) -------------------- */
#define MT_MAX_CHANNELS 8
typedef struct {
    char name[12];
    uint8_t psk[32];
    uint8_t psk_len;
    uint32_t position_precision;
    bool uplink, downlink;   /* MQTT */
} mt_chset_ch_t;

typedef struct {
    uint8_t n;
    mt_chset_ch_t ch[MT_MAX_CHANNELS];
    bool has_lora;
    bool use_preset;
    uint32_t preset;
    uint32_t region;
    uint32_t hop_limit;
    bool tx_enabled;
    int32_t tx_power;
    uint32_t channel_num;
} mt_channelset_t;

bool mt_channelset_decode(const uint8_t *buf, size_t n, mt_channelset_t *out);
size_t mt_channelset_encode(const mt_channelset_t *cs, uint8_t *buf, size_t cap);

/* ---- Routing (ROUTING_APP payload) --------------------------------------- */
typedef struct {
    bool has_error;
    uint32_t error_reason; /* 0 = NONE (ACK) */
} mt_routing_t;

bool mt_routing_decode(const uint8_t *buf, size_t n, mt_routing_t *out);
size_t mt_routing_encode_error(uint32_t reason, uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif
