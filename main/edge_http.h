// Local HTTP API, WebSocket event stream and the built-in setup page.
//
//   GET  /                      setup page
//   POST /api/login             {"pin":"12345678","unix_ms":...} -> {"token":...}
//   POST /api/logout            (Bearer)
//   POST /api/cmd               (Bearer) one JSON command, same as over BLE
//   GET  /api/records?from=&limit=&fmt=json|bin   (Bearer) bulk export
//   POST /api/ota               (Bearer) firmware image; optional X-SHA256 header
//   GET  /ws?token=...          WebSocket: commands in, events out
//
// Sessions live in RAM and end at reboot or when the PIN changes. Five wrong
// PINs lock the login for a minute.
#pragma once

#include <stddef.h>

#include "esp_err.h"

esp_err_t edge_http_start(void);
/** Sends a text frame to one WebSocket client (fd >= 0) or every authenticated client (fd < 0). */
void edge_http_ws_send(int fd, const char *data, size_t len);
int edge_http_ws_count(void);
/** Drops every session (after a PIN change). */
void edge_http_logout_all(void);
