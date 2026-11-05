// Thin MQTT client wrapper (esp-mqtt) for the Meshtastic MQTT module. The
// mesh task owns all Meshtastic logic; this file only connects, subscribes,
// publishes and queues incoming messages for the mesh task to drain.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

#define MQTT_TOPIC_MAX   96
#define MQTT_RX_MAX      420
#define MQTT_SUBS_MAX    10

typedef struct {
    char host[64];
    uint16_t port;
    bool tls;
    char username[32];
    char password[64];
    char client_id[24];
    int n_subs;
    char subs[MQTT_SUBS_MAX][MQTT_TOPIC_MAX];
} mesh_mqtt_cfg_t;

esp_err_t mesh_mqtt_start(const mesh_mqtt_cfg_t *c);
void mesh_mqtt_stop(void);
bool mesh_mqtt_running(void);
bool mesh_mqtt_connected(void);
bool mesh_mqtt_publish(const char *topic, const void *data, size_t len);
/** Takes one received message, false when none is waiting. */
bool mesh_mqtt_take(char *topic, size_t tcap, uint8_t *buf, size_t cap, size_t *len);
cJSON *mesh_mqtt_status_json(void);
