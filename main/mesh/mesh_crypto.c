#include "mesh_crypto.h"

#include <string.h>

/* ==== X25519 (field arithmetic after TweetNaCl, public domain) =================== */

typedef int64_t gf[16];
static const gf K121665 = {0xDB41, 1};

static void car25519(gf o)
{
    for (int i = 0; i < 16; i++) {
        o[i] += (int64_t)1 << 16;
        int64_t c = o[i] >> 16;
        o[(i + 1) * (i < 15)] += c - 1 + 37 * (c - 1) * (i == 15);
        o[i] -= c * 65536;
    }
}

static void sel25519(gf p, gf q, int b)
{
    int64_t c = ~(int64_t)(b - 1);
    for (int i = 0; i < 16; i++) {
        int64_t t = c & (p[i] ^ q[i]);
        p[i] ^= t;
        q[i] ^= t;
    }
}

static void pack25519(uint8_t *o, const gf n)
{
    gf m, t;
    for (int i = 0; i < 16; i++) t[i] = n[i];
    car25519(t);
    car25519(t);
    car25519(t);
    for (int j = 0; j < 2; j++) {
        m[0] = t[0] - 0xffed;
        for (int i = 1; i < 15; i++) {
            m[i] = t[i] - 0xffff - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xffff;
        }
        m[15] = t[15] - 0x7fff - ((m[14] >> 16) & 1);
        int b = (int)((m[15] >> 16) & 1);
        m[14] &= 0xffff;
        sel25519(t, m, 1 - b);
    }
    for (int i = 0; i < 16; i++) {
        o[2 * i] = (uint8_t)(t[i] & 0xff);
        o[2 * i + 1] = (uint8_t)(t[i] >> 8);
    }
}

static void unpack25519(gf o, const uint8_t *n)
{
    for (int i = 0; i < 16; i++) o[i] = n[2 * i] + ((int64_t)n[2 * i + 1] << 8);
    o[15] &= 0x7fff;
}

static void fadd(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) o[i] = a[i] + b[i];
}

static void fsub(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) o[i] = a[i] - b[i];
}

static void fmul(gf o, const gf a, const gf b)
{
    int64_t t[31];
    for (int i = 0; i < 31; i++) t[i] = 0;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) t[i + j] += a[i] * b[j];
    }
    for (int i = 0; i < 15; i++) t[i] += 38 * t[i + 16];
    for (int i = 0; i < 16; i++) o[i] = t[i];
    car25519(o);
    car25519(o);
}

static void fsq(gf o, const gf a)
{
    fmul(o, a, a);
}

static void inv25519(gf o, const gf in)
{
    gf c;
    for (int a = 0; a < 16; a++) c[a] = in[a];
    for (int a = 253; a >= 0; a--) {
        fsq(c, c);
        if (a != 2 && a != 4) fmul(c, c, in);
    }
    for (int a = 0; a < 16; a++) o[a] = c[a];
}

void mesh_x25519_clamp(uint8_t priv[32])
{
    priv[0] &= 248;
    priv[31] = (uint8_t)((priv[31] & 127) | 64);
}

void mesh_x25519(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32])
{
    uint8_t z[32];
    gf x, a, b, c, d, e, f;
    memcpy(z, scalar, 32);
    mesh_x25519_clamp(z);
    unpack25519(x, point);
    for (int i = 0; i < 16; i++) {
        b[i] = x[i];
        d[i] = a[i] = c[i] = 0;
    }
    a[0] = d[0] = 1;
    for (int i = 254; i >= 0; --i) {
        int r = (z[i >> 3] >> (i & 7)) & 1;
        sel25519(a, b, r);
        sel25519(c, d, r);
        fadd(e, a, c);
        fsub(a, a, c);
        fadd(c, b, d);
        fsub(b, b, d);
        fsq(d, e);
        fsq(f, a);
        fmul(a, c, a);
        fmul(c, b, e);
        fadd(e, a, c);
        fsub(a, a, c);
        fsq(b, a);
        fsub(c, d, f);
        fmul(a, c, K121665);
        fadd(a, a, d);
        fmul(c, c, a);
        fmul(a, d, f);
        fmul(d, b, x);
        fsq(b, e);
        sel25519(a, b, r);
        sel25519(c, d, r);
    }
    inv25519(c, c);
    fmul(a, a, c);
    pack25519(out, a);
    memset(z, 0, sizeof(z));
}

void mesh_x25519_public(uint8_t pub[32], const uint8_t priv[32])
{
    static const uint8_t base[32] = {9};
    mesh_x25519(pub, priv, base);
}

/* ==== SHA-256 ===================================================================== */

static const uint32_t K256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(uint32_t h[8], const uint8_t *p)
{
    uint32_t w[64];
    for (int i = 0; i < 16; i++) {
        w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
    }
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], k = h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = k + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K256[i] + w[i];
        uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        k = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += k;
}

void mesh_sha256(const uint8_t *data, size_t len, uint8_t out[32])
{
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    size_t i = 0;
    for (; i + 64 <= len; i += 64) sha256_block(h, data + i);
    uint8_t tail[128] = {0};
    size_t rem = len - i;
    memcpy(tail, data + i, rem);
    tail[rem] = 0x80;
    size_t tl = rem + 1 + 8 <= 64 ? 64 : 128;
    uint64_t bits = (uint64_t)len * 8;
    for (int k = 0; k < 8; k++) tail[tl - 1 - k] = (uint8_t)(bits >> (8 * k));
    sha256_block(h, tail);
    if (tl == 128) sha256_block(h, tail + 64);
    for (int k = 0; k < 8; k++) {
        out[4 * k] = (uint8_t)(h[k] >> 24);
        out[4 * k + 1] = (uint8_t)(h[k] >> 16);
        out[4 * k + 2] = (uint8_t)(h[k] >> 8);
        out[4 * k + 3] = (uint8_t)h[k];
    }
}

/* ==== AES (encrypt direction only; CCM needs nothing else) ======================== */

static const uint8_t SBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
};

static uint32_t sub_word(uint32_t w)
{
    return (uint32_t)SBOX[w >> 24] << 24 | (uint32_t)SBOX[(w >> 16) & 0xff] << 16 |
           (uint32_t)SBOX[(w >> 8) & 0xff] << 8 | SBOX[w & 0xff];
}

void mesh_aes_init(mesh_aes_t *ctx, const uint8_t *key, size_t key_len)
{
    int nk = (int)(key_len / 4);
    ctx->rounds = nk + 6;
    int total = 4 * (ctx->rounds + 1);
    for (int i = 0; i < nk; i++) {
        ctx->rk[i] = (uint32_t)key[4 * i] << 24 | (uint32_t)key[4 * i + 1] << 16 | (uint32_t)key[4 * i + 2] << 8 | key[4 * i + 3];
    }
    uint32_t rcon = 0x01;
    for (int i = nk; i < total; i++) {
        uint32_t t = ctx->rk[i - 1];
        if (i % nk == 0) {
            t = sub_word((t << 8) | (t >> 24)) ^ (rcon << 24);
            rcon = (rcon << 1) ^ ((rcon & 0x80) ? 0x11b : 0);
        } else if (nk > 6 && i % nk == 4) {
            t = sub_word(t);
        }
        ctx->rk[i] = ctx->rk[i - nk] ^ t;
    }
}

static uint8_t xtime(uint8_t x)
{
    return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0));
}

void mesh_aes_encrypt_block(const mesh_aes_t *ctx, const uint8_t in[16], uint8_t out[16])
{
    uint8_t s[16];
    for (int c = 0; c < 4; c++) {
        uint32_t k = ctx->rk[c];
        for (int r = 0; r < 4; r++) s[4 * c + r] = in[4 * c + r] ^ (uint8_t)(k >> (24 - 8 * r));
    }
    for (int round = 1; round <= ctx->rounds; round++) {
        uint8_t t[16];
        /* SubBytes + ShiftRows */
        for (int c = 0; c < 4; c++) {
            for (int r = 0; r < 4; r++) t[4 * c + r] = SBOX[s[4 * ((c + r) % 4) + r]];
        }
        if (round != ctx->rounds) { /* MixColumns */
            for (int c = 0; c < 4; c++) {
                uint8_t *a = &t[4 * c];
                uint8_t a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3], x = a0 ^ a1 ^ a2 ^ a3;
                a[0] ^= x ^ xtime(a0 ^ a1);
                a[1] ^= x ^ xtime(a1 ^ a2);
                a[2] ^= x ^ xtime(a2 ^ a3);
                a[3] ^= x ^ xtime(a3 ^ a0);
            }
        }
        for (int c = 0; c < 4; c++) {
            uint32_t k = ctx->rk[4 * round + c];
            for (int r = 0; r < 4; r++) s[4 * c + r] = t[4 * c + r] ^ (uint8_t)(k >> (24 - 8 * r));
        }
    }
    memcpy(out, s, 16);
}

/* ==== AES-CCM (RFC 3610), L = 2 =================================================== */

static void ccm_mac(const mesh_aes_t *aes, const uint8_t nonce[13], size_t m, const uint8_t *aad, size_t aad_len,
                    const uint8_t *data, size_t len, uint8_t x[16])
{
    uint8_t b[16];
    b[0] = (uint8_t)((aad_len ? 0x40 : 0) | (((m - 2) / 2) << 3) | 1);
    memcpy(b + 1, nonce, 13);
    b[14] = (uint8_t)(len >> 8);
    b[15] = (uint8_t)len;
    mesh_aes_encrypt_block(aes, b, x);
    if (aad_len) {
        /* aad_len < 0xff00 here: two-byte length prefix */
        uint8_t blk[16] = {0};
        size_t pos = 2, used = 0;
        blk[0] = (uint8_t)(aad_len >> 8);
        blk[1] = (uint8_t)aad_len;
        while (used < aad_len) {
            size_t n = aad_len - used < 16 - pos ? aad_len - used : 16 - pos;
            memcpy(blk + pos, aad + used, n);
            used += n;
            pos += n;
            if (pos == 16 || used == aad_len) {
                for (int i = 0; i < 16; i++) x[i] ^= blk[i];
                mesh_aes_encrypt_block(aes, x, x);
                memset(blk, 0, sizeof(blk));
                pos = 0;
            }
        }
    }
    for (size_t off = 0; off < len; off += 16) {
        size_t n = len - off < 16 ? len - off : 16;
        for (size_t i = 0; i < n; i++) x[i] ^= data[off + i];
        mesh_aes_encrypt_block(aes, x, x);
    }
}

static void ccm_ctr(const mesh_aes_t *aes, const uint8_t nonce[13], const uint8_t *in, size_t len, uint8_t *out,
                    uint8_t s0[16])
{
    uint8_t a[16], s[16];
    a[0] = 1; /* L - 1 */
    memcpy(a + 1, nonce, 13);
    a[14] = a[15] = 0;
    mesh_aes_encrypt_block(aes, a, s0);
    uint16_t ctr = 1;
    for (size_t off = 0; off < len; off += 16, ctr++) {
        a[14] = (uint8_t)(ctr >> 8);
        a[15] = (uint8_t)ctr;
        mesh_aes_encrypt_block(aes, a, s);
        size_t n = len - off < 16 ? len - off : 16;
        for (size_t i = 0; i < n; i++) out[off + i] = in[off + i] ^ s[i];
    }
}

void mesh_ccm_encrypt(const uint8_t *key, size_t key_len, const uint8_t nonce[13], size_t m,
                      const uint8_t *aad, size_t aad_len, const uint8_t *plain, size_t len,
                      uint8_t *crypt, uint8_t *tag)
{
    mesh_aes_t aes;
    uint8_t x[16], s0[16];
    mesh_aes_init(&aes, key, key_len);
    ccm_mac(&aes, nonce, m, aad, aad_len, plain, len, x);
    ccm_ctr(&aes, nonce, plain, len, crypt, s0);
    for (size_t i = 0; i < m; i++) tag[i] = x[i] ^ s0[i];
}

bool mesh_ccm_decrypt(const uint8_t *key, size_t key_len, const uint8_t nonce[13], size_t m,
                      const uint8_t *aad, size_t aad_len, const uint8_t *crypt, size_t len,
                      const uint8_t *tag, uint8_t *plain)
{
    mesh_aes_t aes;
    uint8_t x[16], s0[16];
    mesh_aes_init(&aes, key, key_len);
    ccm_ctr(&aes, nonce, crypt, len, plain, s0);
    ccm_mac(&aes, nonce, m, aad, aad_len, plain, len, x);
    uint8_t diff = 0;
    for (size_t i = 0; i < m; i++) diff |= (uint8_t)(tag[i] ^ x[i] ^ s0[i]);
    if (diff) memset(plain, 0, len);
    return diff == 0;
}

/* ==== Meshtastic PKI ============================================================== */

void mesh_pki_shared_key(uint8_t key[32], const uint8_t our_priv[32], const uint8_t their_pub[32])
{
    uint8_t shared[32];
    mesh_x25519(shared, our_priv, their_pub);
    mesh_sha256(shared, 32, key);
    memset(shared, 0, sizeof(shared));
}

static void le32(uint8_t *o, uint32_t v)
{
    o[0] = (uint8_t)v;
    o[1] = (uint8_t)(v >> 8);
    o[2] = (uint8_t)(v >> 16);
    o[3] = (uint8_t)(v >> 24);
}

static void pki_nonce(uint8_t n[13], uint32_t from, uint32_t packet_id, uint32_t extra)
{
    le32(n, packet_id);
    le32(n + 4, extra);
    le32(n + 8, from);
    n[12] = 0;
}

void mesh_pki_encrypt(const uint8_t key[32], uint32_t from, uint32_t packet_id, uint32_t extra_nonce,
                      const uint8_t *plain, size_t len, uint8_t *out)
{
    uint8_t nonce[13];
    pki_nonce(nonce, from, packet_id, extra_nonce);
    mesh_ccm_encrypt(key, 32, nonce, 8, NULL, 0, plain, len, out, out + len);
    le32(out + len + 8, extra_nonce);
}

bool mesh_pki_decrypt(const uint8_t key[32], uint32_t from, uint32_t packet_id, const uint8_t *in, size_t len,
                      uint8_t *plain)
{
    if (len <= MESH_PKI_OVERHEAD) return false;
    size_t n = len - MESH_PKI_OVERHEAD;
    const uint8_t *e = in + n + 8;
    uint32_t extra = (uint32_t)e[0] | (uint32_t)e[1] << 8 | (uint32_t)e[2] << 16 | (uint32_t)e[3] << 24;
    uint8_t nonce[13];
    pki_nonce(nonce, from, packet_id, extra);
    return mesh_ccm_decrypt(key, 32, nonce, 8, NULL, 0, in, n, in + n, plain);
}
