// Mesh module settings kept apart from edge_config_t (own NVS blob "meshmod"),
// so the main config struct stays small and needs no migration: remote admin
// and remote hardware security, store & forward server, MQTT and audio.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define EDGE_MOD_VERSION       2
#define EDGE_ADMIN_KEYS_MAX    3
#define EDGE_RHW_PINS_MAX      8

typedef enum {
    EDGE_ADMIN_OPEN = 0, /* any node may administer this Edge (default; the page warns about it) */
    EDGE_ADMIN_KEYS = 1, /* only PKI messages from a listed admin key (or the legacy "admin" channel when allowed) */
} edge_admin_mode_t;

/* Meshtastic ModuleConfig.RemoteHardwarePinType */
#define EDGE_RHW_READ  1
#define EDGE_RHW_WRITE 2

typedef struct {
    uint8_t gpio;
    uint8_t type;      /* EDGE_RHW_READ / EDGE_RHW_WRITE */
    char name[16];
} edge_rhw_pin_t;

typedef struct {
    uint16_t version;

    /* Remote admin (AdminMessage, port 6) */
    bool admin_enabled;
    uint8_t admin_mode;             /* edge_admin_mode_t */
    bool admin_channel;             /* accept the legacy channel named "admin" */
    uint8_t n_admin_keys;
    uint8_t admin_keys[EDGE_ADMIN_KEYS_MAX][32];

    /* Remote hardware (HardwareMessage, port 2); same sender policy as admin */
    bool rhw_enabled;
    bool rhw_undefined_ok;          /* allow pins not listed below (protected board pins never) */
    uint8_t n_pins;
    edge_rhw_pin_t pins[EDGE_RHW_PINS_MAX];

    /* Store & forward */
    bool sf_enabled;                /* answer as a server (flash-backed history) */
    bool sf_heartbeat;
    uint16_t sf_return_max;         /* messages per history request */
    uint16_t sf_return_window_min;  /* how far back a request looks */

    /* MQTT */
    bool mqtt_enabled;
    char mqtt_address[64];          /* host or host:port */
    char mqtt_username[32];
    char mqtt_password[64];
    bool mqtt_encryption;           /* publish packets still encrypted (Meshtastic default) */
    bool mqtt_json;
    bool mqtt_tls;
    char mqtt_root[32];             /* "" = msh/<region> */
    bool mqtt_proxy_to_client;
    bool mqtt_map;
    uint32_t mqtt_map_interval_s;
    uint8_t mqtt_map_precision;
    bool mqtt_map_location;
    uint8_t ch_uplink;              /* bit per channel index */
    uint8_t ch_downlink;

    /* Audio (codec2 over I2S; needs a microphone, speaker and PTT button) */
    bool audio_enabled;
    int8_t audio_ptt;
    uint8_t audio_bitrate;          /* Meshtastic Audio_Baud */
    int8_t i2s_ws, i2s_sd, i2s_din, i2s_sck;

    /* ATAK (version 2): team colour and member role in TAK position reports (Meshtastic TAKConfig) */
    uint8_t tak_team;  /* Meshtastic Team, 0 = default (Cyan) */
    uint8_t tak_role;  /* Meshtastic MemberRole, 0 = default (Team Member) */
} edge_mesh_mod_t;

void edge_mod_defaults(edge_mesh_mod_t *m);
void edge_mod_get(edge_mesh_mod_t *out);
/** Clamps and persists. */
esp_err_t edge_mod_set(const edge_mesh_mod_t *in);
/** True for GPIOs the board itself uses (LoRa, OLED, USB, flash, button, LED, battery). */
bool edge_pin_protected(int gpio);
