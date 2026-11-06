#include "edge_util.h"

#include <stdio.h>

#include <string.h>

static const char HEX[] = "0123456789abcdef";

void edge_hex(const uint8_t *in, size_t n, char *out)
{
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = HEX[in[i] >> 4];
        out[2 * i + 1] = HEX[in[i] & 0x0f];
    }
    out[2 * n] = '\0';
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool edge_unhex(const char *in, uint8_t *out, size_t n)
{
    if (!in || strlen(in) != 2 * n) return false;
    for (size_t i = 0; i < n; i++) {
        int hi = hexval(in[2 * i]);
        int lo = hexval(in[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

void edge_mac_str(const uint8_t mac[6], char out[18])
{
    for (int i = 0; i < 6; i++) {
        out[i * 3] = HEX[mac[i] >> 4];
        out[i * 3 + 1] = HEX[mac[i] & 0x0f];
        out[i * 3 + 2] = (i == 5) ? '\0' : ':';
    }
}

bool edge_mac_parse(const char *s, uint8_t mac[6])
{
    if (!s) return false;
    int idx = 0;
    int nibble = -1;
    for (const char *p = s; *p; p++) {
        if (*p == ':' || *p == '-') {
            if (nibble >= 0) return false; /* separator in the middle of a byte */
            continue;
        }
        int v = hexval(*p);
        if (v < 0 || idx >= 6) return false;
        if (nibble < 0) {
            nibble = v;
        } else {
            mac[idx++] = (uint8_t)((nibble << 4) | v);
            nibble = -1;
        }
    }
    return idx == 6 && nibble < 0;
}

static int b64val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+' || c == '-') return 62;
    if (c == '/' || c == '_') return 63;
    return -1;
}

int edge_b64_decode(const char *in, uint8_t *out, size_t out_max)
{
    if (!in) return -1;
    uint32_t acc = 0;
    int bits = 0;
    size_t n = 0;
    for (const char *p = in; *p; p++) {
        if (*p == '=') break;
        if (*p == '\n' || *p == '\r' || *p == ' ') continue;
        int v = b64val(*p);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (n >= out_max) return -1;
            out[n++] = (uint8_t)(acc >> bits);
        }
    }
    return (int)n;
}

int edge_b64_encode(const uint8_t *in, size_t n, char *out, size_t out_max)
{
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t need = ((n + 2) / 3) * 4 + 1;
    if (need > out_max) return -1;
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < n) v |= in[i + 2];
        out[o++] = T[(v >> 18) & 63];
        out[o++] = T[(v >> 12) & 63];
        out[o++] = (i + 1 < n) ? T[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < n) ? T[v & 63] : '=';
    }
    out[o] = '\0';
    return (int)o;
}

size_t edge_trim_zeros(const uint8_t *body, size_t n)
{
    while (n > 0 && body[n - 1] == 0) n--;
    return n;
}

int edge_live_row(char *out, size_t cap, const char *kind, const uint8_t addr[6], int rssi_last, int rssi_max,
                  int64_t ago_ms, uint32_t count)
{
    char m[18];
    edge_mac_str(addr, m);
    if (ago_ms < 0) ago_ms = 0;
    int n = snprintf(out, cap, "[\"%s\",\"%s\",%d,%d,%lld,%lu]", kind, m, rssi_last, rssi_max, (long long)ago_ms,
                     (unsigned long)count);
    return n < 0 || (size_t)n >= cap ? -1 : n;
}

uint32_t edge_fnv1a(const uint8_t *salt, size_t salt_len, const uint8_t *data, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < salt_len; i++) {
        h ^= salt[i];
        h *= 16777619u;
    }
    for (size_t i = 0; i < n; i++) {
        h ^= data[i];
        h *= 16777619u;
    }
    return h;
}

void edge_strlcpy(char *dst, const char *src, size_t dst_size)
{
    if (dst_size == 0) return;
    size_t i = 0;
    if (src) {
        for (; i + 1 < dst_size && src[i]; i++) dst[i] = src[i];
    }
    dst[i] = '\0';
}

/* Length of a valid UTF-8 sequence starting at s[0] (max n bytes), or 0. */
static size_t utf8_seq_len(const uint8_t *s, size_t n)
{
    uint8_t c = s[0];
    size_t len;
    if (c >= 0xc2 && c <= 0xdf) len = 2;
    else if (c >= 0xe0 && c <= 0xef) len = 3;
    else if (c >= 0xf0 && c <= 0xf4) len = 4;
    else return 0;
    if (len > n) return 0;
    for (size_t i = 1; i < len; i++) {
        if ((s[i] & 0xc0) != 0x80) return 0;
    }
    return len;
}

void edge_sanitize_text(char *dst, size_t dst_size, const uint8_t *src, size_t n)
{
    if (dst_size == 0) return;
    size_t o = 0;
    size_t i = 0;
    while (i < n && o + 1 < dst_size) {
        uint8_t c = src[i];
        if (c == 0) break;
        if (c >= 0x80) {
            size_t l = utf8_seq_len(&src[i], n - i);
            if (l == 0) {
                dst[o++] = '?';
                i++;
            } else {
                if (o + l + 1 > dst_size) break; /* never cut a character in half */
                memcpy(&dst[o], &src[i], l);
                o += l;
                i += l;
            }
            continue;
        }
        dst[o++] = (c < 0x20 || c == 0x7f) ? (c == '\n' ? ' ' : '?') : (char)c;
        i++;
    }
    dst[o] = '\0';
}
