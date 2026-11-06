// Live hand-off of records to the paired phone.
//
// While the Sotreus app has asked for it (`stream_start` over Bluetooth), every
// new record goes to the phone on the records characteristic instead of into
// flash. The phone confirms with `stream_ack` (cumulative: "everything up to
// seq N"). Records not yet confirmed wait in a small RAM buffer; they are
// written to flash, in order, when the buffer fills, when the phone stops
// confirming for 30 s, or when the link drops or the app stops the stream.
// Nothing is lost, and the flash log keeps increasing sequence numbers.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

void edge_stream_init(void);
/** Starts the hand-off on a secured Bluetooth link subscribed to records. */
esp_err_t edge_stream_start(int conn);
/** Stops it; unconfirmed records are written to flash. */
void edge_stream_stop(void);
/** The phone has every record up to and including seq. */
void edge_stream_ack(uint32_t seq);
/** Called when a Bluetooth link drops or unsubscribes from records. */
void edge_stream_link_down(int conn);
bool edge_stream_active(void);
cJSON *edge_stream_status_json(void);
