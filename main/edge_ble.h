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
#include "edge_log.h"

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
/** Diagnostic (USB console): cap the link-layer payload the Edge sends; 0 restores the default. */
void edge_ble_set_tx_octets(uint16_t octets);
/**
 * Full-size (251-octet) link packets for the bonded phone on `conn`, remembered across links and
 * restarts. `set` < 0 only reads; 0 or 1 stores. `*stored` is the phone's setting, `*this_link`
 * whether this link has (or, while its settled step is pending, will keep) full-size packets.
 * ESP_ERR_NOT_FOUND: no such link. ESP_ERR_INVALID_STATE: the phone is not bonded.
 * ESP_FAIL: applied for this boot but not saved.
 */
esp_err_t edge_ble_full_size(int conn, int set, bool *stored, bool *this_link);

/**
 * Streams records with seq >= from (at most `limit`) on the records
 * characteristic. Blocks until done. Returns the number sent, -1 on error.
 * `*next` receives the seq to resume from.
 */
int edge_ble_stream_records(int conn, uint32_t from, uint32_t limit, uint32_t *next);

/** Sends one record on the records characteristic. 0 = sent, -1 = link gone or not subscribed. */
int edge_ble_notify_record(int conn, const edge_rec_t *rec);
/** True when conn is a secured link subscribed to the records characteristic. */
bool edge_ble_records_ready(int conn);

/** Accept new pairings for `seconds`. */
void edge_ble_open_pairing(uint32_t seconds);
/**
 * Before a planned restart: stops advertising and ends every link, so phones see the drop at once
 * instead of after their supervision timeout. Waits up to `wait_ms` for the links to close.
 */
void edge_ble_close(uint32_t wait_ms);

/** Removes every bond and reopens pairing. */
esp_err_t edge_ble_forget_all(void);

void edge_ble_get_info(edge_ble_info_t *out);
