// Append-only ring of fixed 256-byte records on the "obslog" flash partition.
//
// Each record has a sequence number that never repeats on this device (the
// floor survives log erases in NVS). (device_id, seq) is the stable Edge
// record ID the phone uses to sync without duplicates.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define EDGE_REC_SIZE       256
#define EDGE_REC_BODY_MAX   224
#define EDGE_REC_FORMAT     1

typedef enum {
    EDGE_REC_BLE = 1,        /* BLE advertiser sighting */
    EDGE_REC_WIFI_AP = 2,    /* Wi-Fi access point sighting */
    EDGE_REC_WIFI_PROBE = 3, /* Wi-Fi client probe-request sighting */
    EDGE_REC_MESH_TEXT = 4,  /* LoRa mesh text message (rx or tx) */
    EDGE_REC_EVENT = 5,      /* Edge status event (boot, mode change, ...) */
} edge_rec_type_t;

#define EDGE_RECF_TIME_VALID 0x01 /* first_ms/last_ms are unix ms; otherwise uptime ms of `boot` */
#define EDGE_RECF_CONTINUES  0x02 /* window closed at max_window while still heard */
#define EDGE_RECF_TRUNCATED  0x04 /* body content was cut to fit */

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint8_t type;
    uint8_t flags;
    uint16_t boot;          /* boot count (low 16 bits) when written */
    int64_t first_ms;
    int64_t last_ms;
    uint16_t body_len;
    uint16_t format;        /* EDGE_REC_FORMAT */
    uint8_t body[EDGE_REC_BODY_MAX];
    uint32_t crc;           /* CRC-32 of bytes 0..251 (same value as zlib crc32 / java.util.zip.CRC32) */
} edge_rec_t;

_Static_assert(sizeof(edge_rec_t) == EDGE_REC_SIZE, "record must be 256 bytes");

typedef struct {
    uint32_t head_seq;    /* newest seq, 0 if empty */
    uint32_t oldest_seq;  /* oldest seq still stored, 0 if empty */
    uint32_t count;       /* records currently stored (approx. if there were write gaps) */
    uint32_t capacity;
    uint32_t written;     /* written since boot */
    uint32_t failed;      /* failed writes since boot */
    bool erasing;
} edge_log_stats_t;

esp_err_t edge_log_init(void);

/**
 * Appends a record. Caller fills type, flags (except TIME_VALID), body_len,
 * body, and first_ms/last_ms as *uptime* ms. The log converts times to unix
 * ms when wall time is known, and fills seq, boot, format and crc.
 * On success rec->seq holds the assigned sequence number.
 */
esp_err_t edge_log_append(edge_rec_t *rec);

/** Reads up to `max` records with seq >= from_seq, oldest first. Returns count. */
int edge_log_read_from(uint32_t from_seq, edge_rec_t *out, int max);

void edge_log_stats(edge_log_stats_t *out);

/** Erases every record. With wait=false the erase runs in the background. */
esp_err_t edge_log_erase(bool wait);
