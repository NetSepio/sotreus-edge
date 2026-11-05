// Portable crypto for Meshtastic public-key (PKI) direct messages:
// X25519, SHA-256, AES (encrypt direction) and AES-CCM (M = 8, L = 2).
// Pure C so it runs in the host tests against RFC / FIPS vectors.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** out = X25519(scalar, point). scalar is clamped internally. */
void mesh_x25519(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32]);
/** Public key for a private key (X25519 with the base point 9). */
void mesh_x25519_public(uint8_t pub[32], const uint8_t priv[32]);
/** Clamps random bytes into a valid Curve25519 private key. */
void mesh_x25519_clamp(uint8_t priv[32]);

void mesh_sha256(const uint8_t *data, size_t len, uint8_t out[32]);

typedef struct {
    uint32_t rk[60];
    int rounds;
} mesh_aes_t;

/** key_len 16 or 32. */
void mesh_aes_init(mesh_aes_t *ctx, const uint8_t *key, size_t key_len);
void mesh_aes_encrypt_block(const mesh_aes_t *ctx, const uint8_t in[16], uint8_t out[16]);

/**
 * AES-CCM with a 13-byte nonce (L = 2) and an M-byte tag, as RFC 3610 and
 * Meshtastic's aes_ccm_ae / aes_ccm_ad. aad may be NULL.
 */
void mesh_ccm_encrypt(const uint8_t *key, size_t key_len, const uint8_t nonce[13], size_t m,
                      const uint8_t *aad, size_t aad_len, const uint8_t *plain, size_t len,
                      uint8_t *crypt, uint8_t *tag);
bool mesh_ccm_decrypt(const uint8_t *key, size_t key_len, const uint8_t nonce[13], size_t m,
                      const uint8_t *aad, size_t aad_len, const uint8_t *crypt, size_t len,
                      const uint8_t *tag, uint8_t *plain);

/* ---- Meshtastic PKI framing ---------------------------------------------------
 * payload on air = ciphertext || tag (8 bytes) || extra_nonce (4 bytes LE)
 * key   = SHA-256(X25519(our_private, their_public))
 * nonce = packet_id (u32 LE) || extra_nonce (u32 LE) || from (u32 LE) || 0x00
 */
#define MESH_PKI_OVERHEAD 12

void mesh_pki_shared_key(uint8_t key[32], const uint8_t our_priv[32], const uint8_t their_pub[32]);
/** out must hold len + MESH_PKI_OVERHEAD bytes. */
void mesh_pki_encrypt(const uint8_t key[32], uint32_t from, uint32_t packet_id, uint32_t extra_nonce,
                      const uint8_t *plain, size_t len, uint8_t *out);
/** in holds len bytes including the overhead; returns false on a bad tag. */
bool mesh_pki_decrypt(const uint8_t key[32], uint32_t from, uint32_t packet_id, const uint8_t *in, size_t len,
                      uint8_t *plain);

#ifdef __cplusplus
}
#endif
