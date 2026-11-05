#include "mesh_radio.h"

#include <math.h>
#include <string.h>
#include <strings.h>

/* Region table (subset usable with an SX1262: sub-GHz only). */
static const mesh_region_t REGIONS[] = {
    {1, "US", 902.0f, 928.0f, 100, 0.0f, 30},
    {2, "EU_433", 433.0f, 434.0f, 10, 0.0f, 10},
    {3, "EU_868", 869.4f, 869.65f, 10, 0.0f, 27},
    {4, "CN", 470.0f, 510.0f, 100, 0.0f, 19},
    {5, "JP", 920.5f, 923.5f, 100, 0.0f, 13},
    {6, "ANZ", 915.0f, 928.0f, 100, 0.0f, 30},
    {7, "KR", 920.0f, 923.0f, 100, 0.0f, 23},
    {8, "TW", 920.0f, 925.0f, 100, 0.0f, 27},
    {9, "RU", 868.7f, 869.2f, 100, 0.0f, 20},
    {10, "IN", 865.0f, 867.0f, 100, 0.0f, 30},
    {11, "NZ_865", 864.0f, 868.0f, 100, 0.0f, 36},
    {12, "TH", 920.0f, 925.0f, 100, 0.0f, 16},
    {14, "UA_433", 433.0f, 434.7f, 10, 0.0f, 10},
    {15, "UA_868", 868.0f, 868.6f, 1, 0.0f, 14},
    {16, "MY_433", 433.0f, 435.0f, 100, 0.0f, 20},
    {17, "MY_919", 919.0f, 924.0f, 100, 0.0f, 27},
    {18, "SG_923", 917.0f, 925.0f, 100, 0.0f, 20},
};

static const mesh_preset_t PRESETS[] = {
    {0, "LONG_FAST", "LongFast", 250.0f, 11, 5},
    {1, "LONG_SLOW", "LongSlow", 125.0f, 12, 8},
    {3, "MEDIUM_SLOW", "MediumSlow", 250.0f, 10, 5},
    {4, "MEDIUM_FAST", "MediumFast", 250.0f, 9, 5},
    {5, "SHORT_SLOW", "ShortSlow", 250.0f, 8, 5},
    {6, "SHORT_FAST", "ShortFast", 250.0f, 7, 5},
    {7, "LONG_MODERATE", "LongMod", 125.0f, 11, 8},
    {8, "SHORT_TURBO", "ShortTurbo", 500.0f, 7, 5},
};

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

const mesh_region_t *mesh_region_by_code(uint8_t code)
{
    for (size_t i = 0; i < COUNT(REGIONS); i++) {
        if (REGIONS[i].code == code) return &REGIONS[i];
    }
    return NULL;
}

const mesh_region_t *mesh_region_by_name(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < COUNT(REGIONS); i++) {
        if (strcasecmp(REGIONS[i].name, name) == 0) return &REGIONS[i];
    }
    return NULL;
}

const mesh_region_t *mesh_region_at(size_t i)
{
    return i < COUNT(REGIONS) ? &REGIONS[i] : NULL;
}

const mesh_preset_t *mesh_preset_by_code(uint8_t code)
{
    for (size_t i = 0; i < COUNT(PRESETS); i++) {
        if (PRESETS[i].code == code) return &PRESETS[i];
    }
    return NULL;
}

const mesh_preset_t *mesh_preset_by_name(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < COUNT(PRESETS); i++) {
        if (strcasecmp(PRESETS[i].name, name) == 0 || strcasecmp(PRESETS[i].display, name) == 0) {
            return &PRESETS[i];
        }
    }
    return NULL;
}

const mesh_preset_t *mesh_preset_at(size_t i)
{
    return i < COUNT(PRESETS) ? &PRESETS[i] : NULL;
}

uint32_t mesh_djb2(const char *s)
{
    uint32_t h = 5381;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) h = ((h << 5) + h) + *p;
    return h;
}

uint8_t mesh_xor_hash(const uint8_t *p, size_t n)
{
    uint8_t x = 0;
    for (size_t i = 0; i < n; i++) x ^= p[i];
    return x;
}

uint32_t mesh_num_channels(const mesh_region_t *r, const mesh_preset_t *p)
{
    double width = (double)r->spacing + p->bw_khz / 1000.0;
    double n = floor(((double)r->freq_end - (double)r->freq_start) / width + 1e-6);
    return n < 1 ? 1 : (uint32_t)n;
}

const char *mesh_effective_channel_name(const char *configured, const mesh_preset_t *p)
{
    return (configured && configured[0]) ? configured : p->display;
}

uint32_t mesh_channel_slot(const mesh_region_t *r, const mesh_preset_t *p, const char *channel_name,
                           uint16_t channel_num)
{
    uint32_t n = mesh_num_channels(r, p);
    if (channel_num) return (uint32_t)(channel_num - 1) % n;
    return mesh_djb2(mesh_effective_channel_name(channel_name, p)) % n;
}

float mesh_frequency_mhz(const mesh_region_t *r, const mesh_preset_t *p, const char *channel_name,
                         uint16_t channel_num)
{
    uint32_t slot = mesh_channel_slot(r, p, channel_name, channel_num);
    double bw = p->bw_khz / 1000.0;
    return (float)((double)r->freq_start + bw / 2.0 + slot * bw);
}

static const uint8_t DEFAULT_PSK[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                        0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};

size_t mesh_expand_psk(const uint8_t *psk, size_t psk_len, uint8_t key[32])
{
    memset(key, 0, 32);
    if (psk_len == 0) return 0;
    if (psk_len == 1) {
        if (psk[0] == 0) return 0;
        memcpy(key, DEFAULT_PSK, 16);
        key[15] = (uint8_t)(key[15] + psk[0] - 1);
        return 16;
    }
    if (psk_len > 32) psk_len = 32;
    memcpy(key, psk, psk_len);
    return psk_len <= 16 ? 16 : 32;
}

uint8_t mesh_channel_hash(const char *name, const uint8_t *key, size_t key_len)
{
    return (uint8_t)(mesh_xor_hash((const uint8_t *)name, strlen(name)) ^ mesh_xor_hash(key, key_len));
}

float mesh_symbol_ms(const mesh_preset_t *p)
{
    return (float)(pow(2.0, p->sf) / p->bw_khz);
}

uint32_t mesh_airtime_ms(const mesh_preset_t *p, size_t len)
{
    double tsym = mesh_symbol_ms(p);
    int de = tsym >= 16.0 ? 1 : 0; /* low data rate optimisation */
    double t_pre = (MESH_PREAMBLE_LEN + 4.25) * tsym;
    double num = 8.0 * len - 4.0 * p->sf + 28 + 16 /* CRC */ - 0 /* explicit header */;
    double den = 4.0 * (p->sf - 2 * de);
    double sym = 8 + fmax(ceil(num / den) * (p->cr), 0);
    return (uint32_t)ceil(t_pre + sym * tsym);
}

uint32_t mesh_slot_ms(const mesh_preset_t *p)
{
    /* CAD duration (~2.25 symbols) + propagation + turnaround + MAC processing. */
    return (uint32_t)ceil(2.25 * mesh_symbol_ms(p) + 0.2 + 0.4 + 7.0);
}
