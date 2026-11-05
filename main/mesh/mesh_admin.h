// Meshtastic remote admin (AdminMessage, port 6), mapped onto the Edge's own
// settings. Pure C: it reads and edits copies of edge_config_t and
// edge_mesh_mod_t and reports side effects; mesh.c handles transport,
// sender authorisation and the session passkey.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "edge_config.h"
#include "edge_meshmod.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADM_REPLY_MAX 210

typedef struct {
    uint32_t variant;        /* AdminMessage payload field number, 0 = none / unknown */
    bool is_get;             /* read-only request: no passkey needed */
    bool sensitive;          /* needs a listed admin key (or the admin channel), never allowed in open mode */
    bool has_passkey;
    uint8_t passkey[8];
} adm_class_t;

typedef struct {
    uint32_t my_node;
    const uint8_t *public_key;     /* 32 bytes or NULL */
    const char *fw_version;
    const uint8_t *passkey;        /* 8 bytes, attached to every response */
    bool sensitive_ok;             /* sender may see secrets (MQTT password, channel keys) */
    uint32_t sf_capacity;          /* store & forward records the flash holds */
    /* connection status */
    bool wifi_connected;
    uint32_t ip;                   /* network byte order as Meshtastic sends it (fixed32 of the address) */
    char ssid[33];
    int32_t wifi_rssi;
    bool mqtt_connected;
    bool ble_connected;
} adm_ctx_t;

typedef struct {
    uint8_t reply[ADM_REPLY_MAX];  /* AdminMessage to send back, when reply_len > 0 */
    size_t reply_len;
    bool cfg_changed;
    bool mod_changed;
    bool owner_changed;            /* resend node info */
    int32_t reboot_s;              /* -1 none */
    bool factory_config, factory_device, nodedb_reset;
    uint32_t fav_set, fav_clear, ign_set, ign_clear, remove_node;
    bool set_time;
    uint32_t time;
    bool add_contact;
    uint32_t contact_num;
    char contact_long[40], contact_short[8];
    uint32_t contact_hw;
    bool contact_has_key;
    uint8_t contact_key[32];
    bool wifi_add;
    char wifi_ssid[33], wifi_psk[65];
    bool unsupported;
} adm_result_t;

/** Decodes just enough to decide authorisation. False on malformed input. */
bool mesh_admin_classify(const uint8_t *in, size_t n, adm_class_t *out);

/** Performs the request on cfg / mod (both in-out) and builds any reply. */
bool mesh_admin_apply(const uint8_t *in, size_t n, const adm_ctx_t *ctx, edge_config_t *cfg, edge_mesh_mod_t *mod,
                      adm_result_t *res);

/* ---- Request builders (for tests and for administering other nodes later) ------------ */
size_t mesh_admin_build_get_config(uint32_t type, const uint8_t *passkey, uint8_t *out, size_t cap);
size_t mesh_admin_build_get_module(uint32_t type, const uint8_t *passkey, uint8_t *out, size_t cap);
size_t mesh_admin_build_get_channel(uint32_t index, uint8_t *out, size_t cap);

/* Serial module baud <-> Meshtastic Serial_Baud enum. */
uint32_t mesh_admin_baud_code(uint32_t baud);
uint32_t mesh_admin_baud_value(uint32_t code);

#ifdef __cplusplus
}
#endif
