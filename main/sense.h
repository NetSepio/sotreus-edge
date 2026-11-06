// Awareness pipeline: raw BLE advertising reports and Wi-Fi management
// frames are folded into per-device "sightings" (first/last heard, RSSI
// range, count, latest payload). A sighting is written to the flash log when
// the device goes quiet for close_after_s, or every max_window_s while it is
// still heard. Signal strength is kept as heard; it is proximity evidence
// only and never converted to distance or direction.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t addr[6];     /* display order */
    uint8_t addr_type;   /* 0 public, 1 random */
    uint8_t evt_type;    /* 0..3 advertising PDU, 4 scan response */
    int8_t rssi;
    uint8_t len;
    uint8_t data[31];
} sense_ble_report_t;

typedef struct {
    uint8_t bssid[6];
    uint8_t channel;
    int8_t rssi;
    uint8_t auth;        /* edge_auth_t */
    uint8_t flags;       /* EDGE_APF_* */
    uint16_t beacon_itvl;
    uint8_t ssid_len;
    uint8_t ssid[32];
    uint8_t rid_len;
    uint8_t rid[120];
} sense_ap_report_t;

typedef struct {
    uint8_t mac[6];
    uint8_t channel;
    int8_t rssi;
    uint8_t ssid_len;    /* 0 = wildcard probe */
    uint8_t ssid[32];
} sense_probe_report_t;

typedef struct {
    bool running;
    int ble_open;
    int ap_open;
    int probe_open;
    uint32_t ble_reports;
    uint32_t wifi_frames;
    uint32_t records;
    uint32_t queue_drops;
    uint32_t brief_ble, brief_ap, brief_probe; /* brief sightings pushed out of a full table and only counted */
} sense_stats_t;

typedef enum {
    SENSE_WATCH_BLE = 1,
    SENSE_WATCH_WIFI = 2,
} sense_watch_kind_t;

void sense_init(void);
void sense_start(void);
void sense_stop(void);
bool sense_running(void);
/** Re-reads awareness settings (scan types) while running. */
void sense_reload(void);

/* Feeders: non-blocking, safe from the NimBLE host and Wi-Fi driver tasks. */
void sense_feed_ble(const sense_ble_report_t *r);
void sense_feed_ap(const sense_ap_report_t *r);
void sense_feed_probe(const sense_probe_report_t *r);

void sense_get_stats(sense_stats_t *out);

/** Rows per `live` page (keeps the response small for the heap and for Bluetooth). */
#define SENSE_LIVE_MAX 100

/**
 * The open sightings as a JSON array of `live` rows, strongest last RSSI first, from `offset`
 * (at most `limit`, capped at SENSE_LIVE_MAX). `ago_ms` is measured on the Edge. Caller frees.
 * NULL when awareness is not running or memory is short. `next` is -1 on the last page.
 */
char *sense_live_json(int offset, int limit, int *total, int *next);

/** Streams every report for one device as "rssi" events for `seconds` (max 600). */
esp_err_t sense_watch(sense_watch_kind_t kind, const uint8_t addr[6], uint16_t seconds);
void sense_unwatch(void);
