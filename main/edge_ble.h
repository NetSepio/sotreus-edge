// Bluetooth LE: the app link (GATT service, LE Secure Connections with a
// passkey shown on the Edge's display, bonding) and the observer scan used
// for awareness.
//
// Service  5e0e0001-5307-4e75-8d9a-ed9e5e0e0001
//   info     ...0002  read     static device description (JSON)
//   control  ...0003  write    newline-delimited JSON commands
//   events   ...0004  notify   newline-delimited JSON events and responses
//   records  ...0005  notify   raw 256-byte records during a sync
// Every characteristic requires an encrypted, authenticated (bonded) link.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int connections;
    int secured;          /* connections that completed pairing / encryption */
    int bonds;
    bool pairing_open;
    uint32_t pairing_left_s; /* 0 when open until the first bond */
    bool passkey_active;
    uint32_t passkey;
    bool scanning;
    bool scan_active;
} edge_ble_info_t;

esp_err_t edge_ble_init(void);

void edge_ble_scan_start(bool active);
void edge_ble_scan_stop(void);

/** Notifies a JSON line on the events characteristic (conn < 0: every secured subscriber). */
void edge_ble_send_line(int conn, const char *line, size_t len);
bool edge_ble_has_subscriber(void);

/**
 * Streams records with seq >= from (at most `limit`) on the records
 * characteristic. Blocks until done. Returns the number sent, -1 on error.
 * `*next` receives the seq to resume from.
 */
int edge_ble_stream_records(int conn, uint32_t from, uint32_t limit, uint32_t *next);

/** Accept new pairings for `seconds`. */
void edge_ble_open_pairing(uint32_t seconds);
/** Removes every bond and reopens pairing. */
esp_err_t edge_ble_forget_all(void);

void edge_ble_get_info(edge_ble_info_t *out);
