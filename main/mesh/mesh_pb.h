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
#define MT_PORT_TELEMETRY   67
#define MT_PORT_TRACEROUTE  70

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
} mt_position_t;

bool mt_position_decode(const uint8_t *buf, size_t n, mt_position_t *out);

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
