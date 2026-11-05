// Meshtastic-compatible LoRa mesh node.
//
// On air it follows the Meshtastic 2.x protocol: channel (AES-CTR) and
// public-key (X25519 + AES-CCM) encryption, managed flooding with next-hop
// routing for direct messages, ACK/NAK with retries, roles and rebroadcast
// modes, node info, position, device telemetry, traceroute, neighbor info,
// waypoints and reactions, range test, store-and-forward (client), paxcounter,
// detection sensor, serial text bridge, external notification and canned
// messages. Everything is controlled from the Sotreus app or the Edge's web
// page; the Meshtastic phone API is not provided.
//
// All mesh state lives in RAM only while Mesh mode runs (so Awareness keeps
// the memory) and is persisted to the "meshdb" partition.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"
#include "mesh_pb.h"

typedef struct {
    bool radio_present;
    bool running;
    bool region_set;
    const char *region;   /* NULL when unset */
    const char *preset;
    float freq_mhz;
    uint32_t node_num;
    int nodes;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t relayed;
    float channel_util;   /* percent, last minute */
    char last_from[8];    /* short name or hex of the sender of the last text */
    char last_text[48];
} mesh_summary_t;

esp_err_t mesh_init(void);
void mesh_start(void);
void mesh_stop(void);
void mesh_reload(void);
bool mesh_running(void);

/** to = MT_BROADCAST for the channel. reply_id/emoji: a reply or a reaction (emoji != 0). */
esp_err_t mesh_send_text(uint32_t to, uint8_t channel, const char *text, bool want_ack, uint32_t reply_id,
                         uint32_t emoji, uint32_t *out_id);
esp_err_t mesh_send_canned(int index, uint32_t *out_id);
esp_err_t mesh_traceroute(uint32_t to, uint8_t channel, uint32_t *out_id);
esp_err_t mesh_request_position(uint32_t to, uint8_t channel);
esp_err_t mesh_send_waypoint(const mt_waypoint_t *w, uint8_t channel, uint32_t *out_id);
esp_err_t mesh_sf_request(uint32_t router, uint8_t channel);
esp_err_t mesh_send_nodeinfo_now(void);
/** Latest position handed over by the phone or browser (used in "client" position mode). */
void mesh_set_client_position(int32_t lat_i, int32_t lon_i, int32_t alt);
/** favorite / ignored: 1 set, 0 clear, -1 unchanged. */
esp_err_t mesh_node_flags(uint32_t num, int favorite, int ignored);
/** Writes the node list and messages to flash now (before a restart). */
void mesh_save_now(void);

/* Hooks */
void mesh_pax_seen(bool wifi, const uint8_t mac[6]);      /* from the BLE / Wi-Fi observers */
bool mesh_led(int64_t now_ms, bool *on);                  /* external notification wants the LED */
int mesh_canned_count(void);
bool mesh_canned_get(int index, char *out, size_t cap);

void mesh_get_summary(mesh_summary_t *out);
cJSON *mesh_status_json(void);
cJSON *mesh_nodes_json(void);
cJSON *mesh_messages_json(void);
cJSON *mesh_waypoints_json(void);

/* ATAK: Meshtastic ATAK plugin packets (v1 port 72, v2 port 78). */
esp_err_t mesh_tak_send_chat(const char *text, uint8_t channel);   /* GeoChat to all chat rooms */
esp_err_t mesh_tak_send_position(uint8_t channel);                 /* our position as a TAK report */
cJSON *mesh_tak_json(void);                                        /* TAK users heard */
