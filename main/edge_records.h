// Body layouts for each record type (little-endian, packed) and JSON views.
// docs/protocol.md documents the same layouts for the app.
#pragma once

#include <stdint.h>

#include "cJSON.h"
#include "edge_log.h"

/* ---- EDGE_REC_BLE -------------------------------------------------------- */
#define EDGE_BLEF_SCAN_RSP     0x02 /* rsp[] holds a scan response */
#define EDGE_BLEF_CONNECTABLE  0x04

typedef struct __attribute__((packed)) {
    uint8_t addr[6];        /* display order (MSB first) */
    uint8_t addr_type;      /* 0 public, 1 random */
    uint8_t evt_type;       /* last advertising PDU: 0 ADV_IND, 1 DIRECT_IND, 2 SCAN_IND, 3 NONCONN_IND */
    int8_t rssi_last;
    int8_t rssi_max;
    int8_t rssi_min;
    uint8_t flags;
    uint32_t count;         /* advertising reports heard in this window */
    uint8_t adv_len;
    uint8_t rsp_len;
    uint8_t adv[31];
    uint8_t rsp[31];
} edge_body_ble_t;

/* ---- EDGE_REC_WIFI_AP ---------------------------------------------------- */
#define EDGE_APF_HIDDEN        0x01
#define EDGE_APF_PROBE_RESP    0x02 /* heard in a probe response */
#define EDGE_APF_REMOTE_ID     0x04 /* carries an ASTM F3411 / ASD-STAN Remote ID vendor element */
#define EDGE_APF_FROM_SCAN     0x08 /* only seen through a scan list, no element detail */

typedef enum {
    EDGE_AUTH_OPEN = 0,
    EDGE_AUTH_WEP = 1,
    EDGE_AUTH_WPA = 2,
    EDGE_AUTH_WPA2 = 3,
    EDGE_AUTH_WPA3 = 4,
    EDGE_AUTH_WPA2_WPA3 = 5,
    EDGE_AUTH_ENTERPRISE = 6,
    EDGE_AUTH_OWE = 7,
    EDGE_AUTH_UNKNOWN = 255,
} edge_auth_t;

typedef struct __attribute__((packed)) {
    uint8_t bssid[6];
    uint8_t channel;
    int8_t rssi_last;
    int8_t rssi_max;
    int8_t rssi_min;
    uint8_t auth;           /* edge_auth_t */
    uint8_t flags;
    uint32_t count;
    uint16_t beacon_itvl;   /* TU */
    uint8_t ssid_len;
    uint8_t ssid[32];
    uint8_t rid_len;
    uint8_t rid[120];       /* Remote ID vendor element body (OUI onward), truncated */
} edge_body_wifi_ap_t;

/* ---- EDGE_REC_WIFI_PROBE ------------------------------------------------- */
#define EDGE_PRBF_RANDOMIZED   0x01 /* locally administered (randomised) MAC */
#define EDGE_PRBF_WILDCARD     0x02 /* sent broadcast probes (no network name) */
#define EDGE_PRBF_PLAIN        0x04 /* plain[] network names are stored */
#define EDGE_PRBF_MORE         0x08 /* asked for more networks than stored */
#define EDGE_PROBE_MAX_SSIDS   4

typedef struct __attribute__((packed)) {
    uint8_t len;
    uint8_t ssid[32];
} edge_ssid_t;

typedef struct __attribute__((packed)) {
    uint8_t mac[6];
    uint8_t channel;
    int8_t rssi_last;
    int8_t rssi_max;
    int8_t rssi_min;
    uint8_t flags;
    uint8_t n_ssids;
    uint32_t count;
    uint32_t ssid_hash[EDGE_PROBE_MAX_SSIDS]; /* FNV-1a(probe_salt || ssid) */
    edge_ssid_t plain[EDGE_PROBE_MAX_SSIDS];
} edge_body_wifi_probe_t;

/* ---- EDGE_REC_MESH_TEXT -------------------------------------------------- */
#define EDGE_MESH_TEXT_MAX 206

typedef struct __attribute__((packed)) {
    uint32_t from;
    uint32_t to;
    uint32_t packet_id;
    uint8_t channel;        /* local channel index */
    uint8_t hops;           /* low nibble hop_limit, high nibble hop_start */
    int8_t rssi;
    int8_t snr_x4;
    uint8_t dir;            /* 0 received, 1 sent */
    uint8_t text_len;
    char text[EDGE_MESH_TEXT_MAX];
} edge_body_mesh_text_t;

/* ---- EDGE_REC_EVENT ------------------------------------------------------ */
typedef enum {
    EDGE_EVT_BOOT = 1,
    EDGE_EVT_MODE = 2,
    EDGE_EVT_TIME_SET = 3,
    EDGE_EVT_LOG_ERASED = 4,
    EDGE_EVT_CONFIG = 5,
    EDGE_EVT_FIRMWARE = 6,
    EDGE_EVT_BRIEF = 7,      /* brief sightings counted, not kept one by one (full table), detail has the counts */
} edge_event_code_t;

typedef struct __attribute__((packed)) {
    uint16_t code;
    uint8_t detail_len;
    char detail[221];
} edge_body_event_t;

_Static_assert(sizeof(edge_body_ble_t) <= EDGE_REC_BODY_MAX, "ble body");
_Static_assert(sizeof(edge_body_wifi_ap_t) <= EDGE_REC_BODY_MAX, "ap body");
_Static_assert(sizeof(edge_body_wifi_probe_t) <= EDGE_REC_BODY_MAX, "probe body");
_Static_assert(sizeof(edge_body_mesh_text_t) <= EDGE_REC_BODY_MAX, "mesh body");
_Static_assert(sizeof(edge_body_event_t) <= EDGE_REC_BODY_MAX, "event body");

/** JSON view of a stored record (caller frees). */
cJSON *edge_rec_to_json(const edge_rec_t *rec);

/** Writes an EDGE_REC_EVENT record and publishes it as a live event. */
void edge_record_event(edge_event_code_t code, const char *detail);

const char *edge_auth_name(uint8_t auth);
