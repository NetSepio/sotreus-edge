// Persistent configuration and device identity (NVS).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define EDGE_CONFIG_VERSION     2
#define EDGE_MAX_WIFI_NETS      5
#define EDGE_MESH_MAX_CHANNELS  8
#define EDGE_CANNED_MAX         8
#define EDGE_IGNORE_MAX         8

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

/* Meshtastic Config.DeviceConfig.RebroadcastMode numbering. */
typedef enum {
    EDGE_REBROADCAST_ALL = 0,               /* relay every packet, decodable or not (Meshtastic default) */
    EDGE_REBROADCAST_ALL_SKIP_DECODING = 1, /* relay everything without looking inside (repeater) */
    EDGE_REBROADCAST_LOCAL_ONLY = 2,        /* relay only packets on our own channels */
    EDGE_REBROADCAST_KNOWN_ONLY = 3,        /* relay only packets from nodes we have heard node info from */
    EDGE_REBROADCAST_NONE = 4,
    EDGE_REBROADCAST_CORE_PORTNUMS_ONLY = 5,/* relay only text, position, node info, routing, telemetry */
} edge_rebroadcast_t;

/* Meshtastic Config.DeviceConfig.Role numbering. */
typedef enum {
    EDGE_ROLE_CLIENT = 0,
    EDGE_ROLE_CLIENT_MUTE = 1,
    EDGE_ROLE_ROUTER = 2,
    EDGE_ROLE_REPEATER = 4,
    EDGE_ROLE_TRACKER = 5,
    EDGE_ROLE_SENSOR = 6,
    EDGE_ROLE_TAK = 7,
    EDGE_ROLE_CLIENT_HIDDEN = 8,
    EDGE_ROLE_LOST_AND_FOUND = 9,
    EDGE_ROLE_TAK_TRACKER = 10,
    EDGE_ROLE_ROUTER_LATE = 11,
} edge_role_t;

typedef enum {
    EDGE_POS_OFF = 0,     /* never send a position (default) */
    EDGE_POS_FIXED = 1,   /* a fixed position entered by the owner */
    EDGE_POS_CLIENT = 2,  /* the latest position the phone or browser handed over */
} edge_pos_mode_t;

typedef struct {
    char name[12];     /* Meshtastic channel name, max 11 chars; "" = modem preset name */
    uint8_t psk[32];
    uint8_t psk_len;   /* 0 = no encryption, 1 = "simple" index, 16 or 32 = AES key */
    uint8_t position_precision; /* bits of position shared on this channel: 0 = none, 32 = full */
} edge_mesh_channel_t;

/* Mesh behaviour beyond the radio basics (config version 2). */
typedef struct {
    uint8_t role;                   /* edge_role_t */
    bool licensed;                  /* amateur radio: no encryption, callsign as long name */
    /* Position */
    uint8_t pos_mode;               /* edge_pos_mode_t */
    int32_t pos_lat_i, pos_lon_i, pos_alt;
    uint32_t pos_interval_s;
    /* Telemetry (device metrics) */
    uint32_t telemetry_interval_s;  /* 0 = never send */
    /* Neighbor info */
    bool neighbor_info;
    uint32_t neighbor_interval_s;
    /* Range test */
    uint32_t range_test_interval_s; /* 0 = not sending */
    /* External notification */
    uint8_t notify_mode;            /* 0 off, 1 any message, 2 bell character only */
    int8_t notify_gpio;             /* extra output pin (buzzer/LED), -1 = on-board LED only */
    /* Detection sensor */
    bool detect_enabled;
    int8_t detect_gpio;
    bool detect_active_high;
    uint16_t detect_min_interval_s;
    char detect_name[20];
    /* Paxcounter */
    bool pax_enabled;
    uint32_t pax_interval_s;
    /* Serial module (text lines <-> mesh) */
    bool serial_enabled;
    int8_t serial_rx, serial_tx;
    uint32_t serial_baud;
    /* Canned messages */
    uint8_t n_canned;
    char canned[EDGE_CANNED_MAX][40];
    /* LoRa extras */
    int32_t freq_offset_hz;
    uint32_t override_freq_khz;     /* 0 = use the region / preset slot */
    bool tx_enabled;                /* false = receive only */
    bool override_duty_cycle;       /* ignore the region's duty-cycle limit (owner's responsibility) */
    bool rx_boosted;
    bool ignore_mqtt;               /* drop packets that came through MQTT gateways */
    bool ok_to_mqtt;                /* let gateways upload our packets (off by default) */
    uint8_t n_ignore;
    uint32_t ignore[EDGE_IGNORE_MAX];
} edge_mesh_ext_t;

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
    edge_mesh_ext_t mx;
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
/** Records with seq at or below this were confirmed by the phone and deleted (0 = none). */
uint32_t edge_log_cut_get(void);
esp_err_t edge_log_cut_set(uint32_t cut);
esp_err_t edge_seq_floor_set(uint32_t floor);

#define EDGE_CTX_MAX 2048
/** Context-page settings: an opaque JSON object owned by the web app. Returns length, 0 if unset. */
size_t edge_ctx_get(char *out, size_t cap);
esp_err_t edge_ctx_set(const char *json);

/** Local time zone offset in minutes from UTC, as reported by a browser or phone. */
bool edge_tz_get(int32_t *minutes);
esp_err_t edge_tz_set(int32_t minutes);

/* ---- Placement (protocol: config.placement) --------------------------------------------
 * How the owner uses this Edge, and where it is when it stays in one place. Stored only: the
 * Edge does not use it, show it, or send it over the mesh. The phone app tags sightings with it. */
#define EDGE_PLACE_VERSION 1

typedef enum {
    EDGE_PLACE_TRAVEL = 0,     /* carried; the phone tags sightings while they are together */
    EDGE_PLACE_STATIONARY = 1, /* left in one place; sightings use the position below */
} edge_place_mode_t;

typedef enum {
    EDGE_PLACE_SRC_NONE = 0,
    EDGE_PLACE_SRC_PHONE = 1,  /* averaged phone fixes taken next to the Edge */
    EDGE_PLACE_SRC_MAP = 2,    /* picked on a map by the owner */
    EDGE_PLACE_SRC_GNSS = 3,   /* the Edge's own receiver (boards with GNSS) */
} edge_place_source_t;

typedef struct {
    uint16_t version;
    uint8_t mode;              /* edge_place_mode_t */
    uint8_t source;            /* edge_place_source_t */
    bool positioned;
    int32_t lat_e7;
    int32_t lon_e7;
    uint32_t accuracy_dm;      /* 0.1 m */
    int64_t set_unix_ms;
} edge_placement_t;

void edge_placement_get(edge_placement_t *out);
esp_err_t edge_placement_set(const edge_placement_t *in);

/** Mesh public/private key pair (Curve25519), created on first use. */
esp_err_t edge_pki_get(uint8_t priv[32], uint8_t pub[32]);
esp_err_t edge_pki_regenerate(void);

/** Erases all NVS (settings, networks, Bluetooth bonds, identity). Caller reboots. */
esp_err_t edge_config_factory_reset(void);
