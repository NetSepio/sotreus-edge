// Live event fan-out. Events are newline-delimited JSON objects ("NDJSON")
// with an "ev" field. Delivery is best-effort: the flash log plus sync is the
// reliable path, live events are for UI responsiveness.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "edge_log.h"

typedef enum {
    EDGE_SINK_ALL = 0, /* every subscribed BLE and WebSocket client */
    EDGE_SINK_BLE = 1, /* a single BLE connection (conn handle in `target`) */
    EDGE_SINK_WS = 2,  /* a single WebSocket client (socket fd in `target`) */
    EDGE_SINK_UART = 3, /* the USB serial console */
} edge_sink_t;

void edge_events_init(void);

/** Broadcasts `ev` (not freed). Cheap no-op when nobody listens. */
void edge_events_publish(const cJSON *ev);

/** Announces a freshly written record: the raw 256 bytes as a binary WebSocket
 *  frame, and {"ev":"obs","seq":N,"type":T} to BLE clients (who fetch it with sync). */
void edge_events_publish_record(const edge_rec_t *rec);

/** Sends `msg` (not freed) to one client. */
void edge_events_send_to(edge_sink_t sink, int target, const cJSON *msg);

bool edge_events_has_listeners(void);
uint32_t edge_events_dropped(void);
