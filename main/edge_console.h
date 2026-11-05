// USB serial console: one JSON command per line (same commands as the BLE
// and HTTP API). Responses are printed as lines starting with "@edge ".
// Physical USB access already implies ownership, so no PIN is asked.
#pragma once

void edge_console_init(void);
