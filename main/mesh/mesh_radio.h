// Meshtastic-compatible LoRa parameters: regions, modem presets, channel
// frequency slot, channel hash, PSK expansion and airtime. Pure C, host-tested.
//
// Values follow the public Meshtastic protocol so this firmware can talk to
// Meshtastic nodes. Sotreus Edge is not affiliated with Meshtastic.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t code;          /* Meshtastic Config.LoRaConfig.RegionCode */
    const char *name;
    float freq_start;      /* MHz */
    float freq_end;        /* MHz */
    uint8_t duty_cycle;    /* percent per hour, 100 = unlimited */
    float spacing;         /* MHz between channels */
    uint8_t power_limit;   /* dBm */
} mesh_region_t;

typedef struct {
    uint8_t code;          /* Meshtastic Config.LoRaConfig.ModemPreset */
    const char *name;      /* API name, e.g. "LONG_FAST" */
    const char *display;   /* channel display name, e.g. "LongFast" */
    float bw_khz;
    uint8_t sf;
    uint8_t cr;            /* 5..8 meaning 4/5..4/8 */
} mesh_preset_t;

#define MESH_SYNC_WORD      0x2b
#define MESH_PREAMBLE_LEN   16
#define MESH_HEADER_LEN     16
#define MESH_MAX_PACKET     255
#define MESH_MAX_PAYLOAD    (MESH_MAX_PACKET - MESH_HEADER_LEN)

const mesh_region_t *mesh_region_by_code(uint8_t code);
const mesh_region_t *mesh_region_by_name(const char *name);
const mesh_region_t *mesh_region_at(size_t i); /* NULL past the end */

const mesh_preset_t *mesh_preset_by_code(uint8_t code);
const mesh_preset_t *mesh_preset_by_name(const char *name);
const mesh_preset_t *mesh_preset_at(size_t i);

uint32_t mesh_djb2(const char *s);
uint8_t mesh_xor_hash(const uint8_t *p, size_t n);

uint32_t mesh_num_channels(const mesh_region_t *r, const mesh_preset_t *p);
/** channel_num: 0 = slot from hash(channel_name), else 1-based slot. */
uint32_t mesh_channel_slot(const mesh_region_t *r, const mesh_preset_t *p, const char *channel_name,
                           uint16_t channel_num);
float mesh_frequency_mhz(const mesh_region_t *r, const mesh_preset_t *p, const char *channel_name,
                         uint16_t channel_num);

/**
 * Expands a configured PSK to the AES key actually used.
 * psk_len 0 -> no encryption; 1 -> default key variant (index 0 = none);
 * otherwise zero-padded to 16 or 32. Returns key length (0, 16 or 32).
 */
size_t mesh_expand_psk(const uint8_t *psk, size_t psk_len, uint8_t key[32]);

/** Meshtastic channel hash: xor(name bytes) ^ xor(expanded key bytes). */
uint8_t mesh_channel_hash(const char *name, const uint8_t *key, size_t key_len);

/** Name used for hashing: configured name, or the preset display name when empty. */
const char *mesh_effective_channel_name(const char *configured, const mesh_preset_t *p);

float mesh_symbol_ms(const mesh_preset_t *p);
/** Time on air for a payload of `len` bytes (explicit header, CRC on). */
uint32_t mesh_airtime_ms(const mesh_preset_t *p, size_t len);
/** Contention slot used for rebroadcast back-off. */
uint32_t mesh_slot_ms(const mesh_preset_t *p);

#ifdef __cplusplus
}
#endif
