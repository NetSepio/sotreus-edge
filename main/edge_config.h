// Persistent configuration and device identity (NVS).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define EDGE_CONFIG_VERSION     1
#define EDGE_MAX_WIFI_NETS      5
#define EDGE_MESH_MAX_CHANNELS  4

typedef enum {
    EDGE_MODE_AWARENESS = 0, /* BLE + Wi-Fi observation */
    EDGE_MODE_MESH = 1,      /* Meshtastic-compatible LoRa mesh, observation paused */
} edge_mode_t;

typedef enum {
    EDGE_LINK_AUTO = 0, /* join a known network if one is in range, otherwise run the access point */
    EDGE_LINK_AP = 1,
    EDGE_LINK_STA = 2,
    EDGE_LINK_OFF = 3,  /* Bluetooth only */
} edge_link_mode_t;

typedef enum {
    EDGE_REBROADCAST_ALL = 0,        /* relay every packet, decodable or not (Meshtastic default) */
    EDGE_REBROADCAST_LOCAL_ONLY = 1, /* relay only packets on our own channels */
    EDGE_REBROADCAST_NONE = 2,
} edge_rebroadcast_t;

typedef struct {
    char name[12];     /* Meshtastic channel name, max 11 chars; "" = modem preset name */
    uint8_t psk[32];
    uint8_t psk_len;   /* 0 = no encryption, 1 = "simple" index, 16 or 32 = AES key */
} edge_mesh_channel_t;

typedef struct {
    uint16_t version;
    uint8_t mode;               /* edge_mode_t */
    uint8_t link_mode;          /* edge_link_mode_t */

    /* Awareness */
    bool ble_scan;
    bool ble_active;            /* send scan requests to get scan responses */
    bool wifi_scan;             /* access points (beacons / probe responses / scans) */
    bool wifi_probes;           /* probe requests from client devices */
    bool probe_ssid_plain;      /* store probed network names in clear (off = salted hash only) */
    int8_t rssi_floor;          /* ignore weaker than this (dBm) */
    uint16_t close_after_s;     /* a sighting closes after this long unheard */
    uint16_t max_window_s;      /* a sighting is written at least this often while heard */

    /* Display */
    uint16_t display_timeout_s; /* 0 = always on */

    /* Mesh */
    uint8_t region;             /* Meshtastic RegionCode; 0 = unset, radio stays silent */
    uint8_t preset;             /* Meshtastic ModemPreset */
    uint8_t hop_limit;          /* 1..7 */
    uint8_t rebroadcast;        /* edge_rebroadcast_t */
    int8_t tx_power;            /* dBm, 0 = region limit capped to the board */
    uint16_t channel_num;       /* 0 = derive slot from channel name hash, else 1-based slot */
    uint32_t node_num;          /* random at first boot, not derived from any MAC */
    char long_name[40];
    char short_name[5];
    uint32_t nodeinfo_interval_s;
    uint8_t n_channels;
    edge_mesh_channel_t channels[EDGE_MESH_MAX_CHANNELS];
} edge_config_t;

typedef struct {
    char ssid[33];
    char password[65];
} edge_wifi_net_t;

typedef struct {
    uint8_t device_id[8];
    char access_pin[9];      /* 8 digits, unlocks the local web page / HTTP API */
    char ap_ssid[33];
    char ap_password[17];
    char suffix[5];          /* 4 hex chars shown on the display and in names */
    uint8_t probe_salt[8];
    uint8_t sta_mac[6];      /* random, locally administered; used instead of the factory MAC */
    uint32_t boot_count;
} edge_identity_t;

esp_err_t edge_config_init(void);
void edge_config_get(edge_config_t *out);
/** Validates, clamps and persists. */
esp_err_t edge_config_set(const edge_config_t *in);
void edge_config_defaults(edge_config_t *cfg);

const edge_identity_t *edge_identity(void);
esp_err_t edge_identity_new_pin(void);

int edge_wifi_nets_get(edge_wifi_net_t *out, int max);
esp_err_t edge_wifi_net_add(const char *ssid, const char *password);
esp_err_t edge_wifi_net_remove(const char *ssid);

uint32_t edge_seq_floor_get(void);
esp_err_t edge_seq_floor_set(uint32_t floor);

/** Erases all NVS (settings, networks, Bluetooth bonds, identity). Caller reboots. */
esp_err_t edge_config_factory_reset(void);
