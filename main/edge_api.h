// JSON command API shared by the BLE control characteristic, the HTTP API
// and the WebSocket. See docs/protocol.md for the command list.
#pragma once

#include "cJSON.h"
#include "edge_events.h"

#define EDGE_PROTOCOL_VERSION 1

void edge_api_init(void);

/** Queues a raw JSON line from a client; takes ownership of `line`. */
void edge_api_submit(edge_sink_t origin, int target, char *line);

/** Handles one request synchronously and returns the response (caller frees). */
cJSON *edge_api_handle(const cJSON *req, edge_sink_t origin, int target);

/** Static description of the device (stable for a boot). */
cJSON *edge_api_info_json(void);
cJSON *edge_api_status_json(void);
