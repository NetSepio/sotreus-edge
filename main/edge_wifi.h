// Wi-Fi link (station / access point / off) and the 2.4 GHz management
// frame sniffer used for awareness.
//
// Link policy (link_mode):
//   auto - join the strongest known network in range; if none is in range (or
//          joining keeps failing) run the Edge's own access point and look for
//          known networks again every 5 minutes while nobody is connected to it.
//   ap   - always run the access point.
//   sta  - only join known networks.
//   off  - Bluetooth only; the radio is used purely for listening.
//
// Sniffing hops across channels 1-13 only when that cannot disturb the link
// (link off, or access point with no connected clients, where the access
// point's own channel is revisited every other step). Otherwise it listens on
// the link's channel and adds a passive scan every 45 s.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

typedef enum {
    EDGE_WIFI_OFF = 0,
    EDGE_WIFI_AP,
    EDGE_WIFI_STA_SEARCHING,
    EDGE_WIFI_STA_CONNECTING,
    EDGE_WIFI_STA_CONNECTED,
    EDGE_WIFI_STA_WAITING,
} edge_wifi_state_t;

typedef struct {
    edge_wifi_state_t state;
    char ssid[33];       /* joined network, or the Edge's own AP name */
    char ip[16];
    int stations;        /* clients on the Edge's AP */
    uint8_t channel;
    bool sniffing;
    bool hopping;
    int8_t rssi;         /* of the joined network */
} edge_wifi_info_t;

esp_err_t edge_wifi_init(void);
/** Link mode or known networks changed. */
void edge_wifi_reconfigure(void);
void edge_wifi_sniff_set(bool aps, bool probes);
void edge_wifi_get_info(edge_wifi_info_t *out);
const char *edge_wifi_state_name(edge_wifi_state_t s);
/** Networks seen in the last scan (for the setup page), strongest first. */
cJSON *edge_wifi_scan_json(void);
/** Starts a scan now (results arrive a couple of seconds later). */
void edge_wifi_request_scan(void);
