// Meshtastic-compatible LoRa mesh node (client role, managed flooding).
//
// Interoperates on air with Meshtastic nodes that share the same region,
// modem preset and channel (name + PSK): text messages, node info, position
// (decode only), routing ACKs and relaying. It does not implement the
// Meshtastic phone API, PKI direct messages, store-and-forward, MQTT or the
// admin/telemetry modules.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

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
    char last_from[8];    /* short name or hex of the sender of the last text */
    char last_text[48];
} mesh_summary_t;

/** Creates the mesh task and probes the radio. Radio stays asleep. */
esp_err_t mesh_init(void);
/** Starts or stops the mesh (asynchronous; the mesh task applies it). */
void mesh_start(void);
void mesh_stop(void);
/** Re-reads the configuration and retunes if running. */
void mesh_reload(void);
bool mesh_running(void);

/**
 * Queues a text message. to = MT_BROADCAST (0xffffffff) for the channel.
 * want_ack applies to direct messages; delivery is reported as "mesh_tx" events.
 */
esp_err_t mesh_send_text(uint32_t to, uint8_t channel, const char *text, bool want_ack, uint32_t *out_id);

void mesh_get_summary(mesh_summary_t *out);
cJSON *mesh_status_json(void);
cJSON *mesh_nodes_json(void);
cJSON *mesh_messages_json(void);
