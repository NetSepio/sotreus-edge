// More Meshtastic messages: MeshPacket + ServiceEnvelope (MQTT), MapReport,
// the full StoreAndForward message (server side) and HardwareMessage.
// Pure C, host-tested.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT_PORT_AUDIO       9
#define MT_PORT_ATAK        72
#define MT_PORT_MAP_REPORT  73
#define MT_PORT_ATAK_V2     78

/* ---- MeshPacket (as carried over MQTT) ----------------------------------------------- */
typedef struct {
    uint32_t from, to, id;
    uint32_t channel;          /* channel hash when encrypted */
    bool decoded;              /* payload is an encoded Data message, else encrypted bytes */
    const uint8_t *payload;
    size_t payload_len;
    uint32_t rx_time;
    float rx_snr;
    int32_t rx_rssi;
    uint32_t hop_limit, hop_start;
    bool want_ack, via_mqtt, pki_encrypted;
    uint32_t next_hop, relay_node;
    uint32_t priority;
} mt_packet_t;

typedef struct {
    mt_packet_t packet;
    bool has_packet;
    char channel_id[16];
    char gateway_id[16];
} mt_envelope_t;

size_t mt_envelope_encode(const mt_envelope_t *e, uint8_t *buf, size_t cap);
/** payload pointers point into buf. */
bool mt_envelope_decode(const uint8_t *buf, size_t n, mt_envelope_t *out);

/* ---- MapReport --------------------------------------------------------------------- */
typedef struct {
    const char *long_name, *short_name, *firmware_version;
    uint32_t role, hw_model, region, modem_preset;
    bool has_default_channel;
    bool has_position;
    int32_t lat_i, lon_i, altitude;
    uint32_t position_precision;
    uint32_t online_nodes;
    bool opted_location;
} mt_mapreport_t;

size_t mt_mapreport_encode(const mt_mapreport_t *m, uint8_t *buf, size_t cap);

/* ---- StoreAndForward (server side) --------------------------------------------------- */
#define MT_SF_ROUTER_ERROR   1
#define MT_SF_ROUTER_PING    3
#define MT_SF_ROUTER_PONG    4
#define MT_SF_ROUTER_BUSY    5
#define MT_SF_ROUTER_STATS   7
#define MT_SF_CLIENT_ERROR   64
#define MT_SF_CLIENT_STATS   66
#define MT_SF_CLIENT_PING    67
#define MT_SF_CLIENT_PONG    68
#define MT_SF_CLIENT_ABORT   106

typedef struct {
    uint32_t messages_total, messages_saved, messages_max, up_time, requests, requests_history;
    bool heartbeat;
    uint32_t return_max, return_window;
} mt_sf_stats_t;

size_t mt_sf_encode_stats(const mt_sf_stats_t *s, uint8_t *buf, size_t cap);
size_t mt_sf_encode_history(uint32_t rr, uint32_t messages, uint32_t window_ms, uint32_t last_request, uint8_t *buf,
                            size_t cap);
size_t mt_sf_encode_heartbeat(uint32_t period, uint32_t secondary, uint8_t *buf, size_t cap);
size_t mt_sf_encode_text(uint32_t rr, const uint8_t *text, size_t len, uint8_t *buf, size_t cap);
size_t mt_sf_encode_rr(uint32_t rr, uint8_t *buf, size_t cap);

/* ---- HardwareMessage (REMOTE_HARDWARE_APP) ------------------------------------------------ */
#define MT_HW_WRITE_GPIOS      1
#define MT_HW_WATCH_GPIOS      2
#define MT_HW_GPIOS_CHANGED    3
#define MT_HW_READ_GPIOS       4
#define MT_HW_READ_GPIOS_REPLY 5

typedef struct {
    uint32_t type;
    uint64_t mask;
    uint64_t value;
} mt_hw_t;

size_t mt_hw_encode(const mt_hw_t *h, uint8_t *buf, size_t cap);
bool mt_hw_decode(const uint8_t *buf, size_t n, mt_hw_t *out);

/* ---- ATAK (TAKPacket on port 72, TAKPacketV2 on port 78) ------------------------------------
 * Port 78 payload = one flags byte + TAKPacketV2. Flags 0xFF means the protobuf follows
 * uncompressed; anything else is zstd-dictionary compressed by the ATAK plugin / apps. */
#define MT_TAK_RAW_FLAGS   0xFF
#define MT_COT_A_F_G_U_C   1   /* friendly ground unit (a position report) */
#define MT_COT_B_T_F       25  /* GeoChat */
#define MT_COT_HOW_H_E     1   /* entered by hand */
#define MT_COT_HOW_M_G     2   /* machine, GPS */
#define MT_GEO_SRC_GPS     1
#define MT_GEO_SRC_USER    2

typedef struct {
    bool v2;
    bool compressed;          /* strings compressed (v1) or whole packet compressed (v2): not readable here */
    uint32_t cot_type, how;
    char callsign[32];
    char device_callsign[32];
    char uid[48];
    uint32_t team, role;
    bool has_pos;
    int32_t lat_i, lon_i, alt;
    uint32_t speed, course, battery;
    uint32_t geo_src, alt_src;
    bool has_chat;
    char chat[201];
    char chat_to[48];
    char chat_to_callsign[32];
} mt_tak_t;

/** TAKPacketV2 protobuf (no flags byte). */
size_t mt_takv2_encode(const mt_tak_t *t, uint8_t *buf, size_t cap);
bool mt_takv2_decode(const uint8_t *buf, size_t n, mt_tak_t *out);
/** Port 78 payload: handles the flags byte. False for malformed input; compressed payloads return true with compressed set. */
bool mt_tak_port78_decode(const uint8_t *buf, size_t n, mt_tak_t *out);
/** TAKPacket (v1, port 72). Coordinates are readable even when strings are compressed. */
bool mt_takv1_decode(const uint8_t *buf, size_t n, mt_tak_t *out);
size_t mt_takv1_encode(const mt_tak_t *t, uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif
