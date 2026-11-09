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
    uint32_t released;    /* records deleted after a confirmed sync since boot */
    bool releasing;
} edge_log_stats_t;

/** Records numbered in `boot` (seq `from`..`to`) that were neither stored nor confirmed when it restarted. */
typedef struct {
    uint32_t boot;
    uint32_t from;
    uint32_t to;
} edge_log_lost_t;

#define EDGE_LOG_LOST_MAX 8

typedef struct {
    uint32_t since_boot;  /* counting runs from this boot; 0 = not counting (log unavailable) */
    uint32_t total;       /* records lost at restarts since then, including ranges no longer listed */
    int n;
    edge_log_lost_t r[EDGE_LOG_LOST_MAX]; /* newest last; only restarts that lost records */
} edge_log_restart_loss_t;

esp_err_t edge_log_init(void);

/**
 * Appends a record. Caller fills type, flags (except TIME_VALID), body_len,
 * body, and first_ms/last_ms as *uptime* ms. The log converts times to unix
 * ms when wall time is known, and fills seq, boot, format and crc.
 * On success rec->seq holds the assigned sequence number.
 */
esp_err_t edge_log_append(edge_rec_t *rec);

/**
 * Live hand-off: when a hook is set and returns true, the record (already given its seq)
 * went to the phone instead of flash. The hook runs while records are ordered, so it must
 * not call edge_log_append itself.
 */
typedef bool (*edge_log_hook_t)(edge_rec_t *rec);
void edge_log_set_hook(edge_log_hook_t hook);
/** Holds back new records while the caller writes handed-off ones to flash in order. */
void edge_log_route_lock(void);
void edge_log_route_unlock(void);
/** Writes a record that already has its seq (call with the route lock held). */
esp_err_t edge_log_write_assigned(edge_rec_t *rec);
/** The seq the next record will get; stable only while the caller holds the route lock. */
uint32_t edge_log_next_seq(void);

/** Reads up to `max` records with seq >= from_seq, oldest first. Returns count. */
int edge_log_read_from(uint32_t from_seq, edge_rec_t *out, int max);

void edge_log_stats(edge_log_stats_t *out);

/**
 * Records lost at restarts. Counted in RTC memory, which survives a software restart, a crash and
 * a watchdog reset; after a power loss or the reset pin (flashing uses it) the count starts again
 * at that boot and the loss at that restart is not known.
 */
void edge_log_restart_loss(edge_log_restart_loss_t *out);
/** The phone confirmed every handed-off record up to `seq` (none of them is in flash). */
void edge_log_delivered(uint32_t seq);

/**
 * The phone has safely stored every record up to `through`: they are no longer served and their
 * flash is erased in the background. `applied` gets the seq actually used (at most the newest).
 */
esp_err_t edge_log_release(uint32_t through, uint32_t *applied);

/** Erases every record. With wait=false the erase runs in the background. */
esp_err_t edge_log_erase(bool wait);
