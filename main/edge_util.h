// Small pure-C helpers shared by the firmware and the host tests.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Lower-case hex. `out` must hold 2*n+1 bytes. */
void edge_hex(const uint8_t *in, size_t n, char *out);

/** Parse exactly 2*n hex digits into `out`. */
bool edge_unhex(const char *in, uint8_t *out, size_t n);

/** "aa:bb:cc:dd:ee:ff" (display order, mac[0] first). `out` holds 18 bytes. */
void edge_mac_str(const uint8_t mac[6], char out[18]);

/** Accepts "aa:bb:cc:dd:ee:ff", "aa-bb-..." or "aabbccddeeff". */
bool edge_mac_parse(const char *s, uint8_t mac[6]);

/** Standard base64 (RFC 4648) with or without padding. Returns length or -1. */
int edge_b64_decode(const char *in, uint8_t *out, size_t out_max);

/** Standard base64 with padding. Returns length (without NUL) or -1. */
int edge_b64_encode(const uint8_t *in, size_t n, char *out, size_t out_max);

/** Length of `body` without its trailing zero bytes (receivers pad with zeros). */
size_t edge_trim_zeros(const uint8_t *body, size_t n);

/** One `live` row: ["kind","AA:BB:..",rssi_last,rssi_max,ago_ms,count]. Returns length or -1 if it does not fit. */
int edge_live_row(char *out, size_t cap, const char *kind, const uint8_t addr[6], int rssi_last, int rssi_max,
                  int64_t ago_ms, uint32_t count);

/** FNV-1a 32-bit over salt || data. */
uint32_t edge_fnv1a(const uint8_t *salt, size_t salt_len, const uint8_t *data, size_t n);

/** Copy at most dst_size-1 bytes and always NUL-terminate. */
void edge_strlcpy(char *dst, const char *src, size_t dst_size);

/** Copy `n` bytes of untrusted text, replacing control characters and invalid UTF-8 with '?'.
 *  UTF-8 multi-byte sequences are kept. Always NUL-terminates. */
void edge_sanitize_text(char *dst, size_t dst_size, const uint8_t *src, size_t n);

#ifdef __cplusplus
}
#endif
