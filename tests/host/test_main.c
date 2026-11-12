// Host tests for the pure-C parts of the firmware: Meshtastic radio math,
// protobuf codec, and helpers. Run with `make -C tests/host`.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "edge_dns.h"
#include "mesh_crypto.h"
#include "edge_util.h"
#include "mesh_pb.h"
#include "mesh_radio.h"
#include "mesh_admin.h"
#include "mesh_pb_ext.h"
#include "pb_wire.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 0.0005)

static void test_frequency(void)
{
    const mesh_preset_t *lf = mesh_preset_by_name("LONG_FAST");
    CHECK(lf && lf->sf == 11 && lf->bw_khz == 250.0f);
    CHECK(mesh_djb2("LongFast") == 130429955u);

    const mesh_region_t *us = mesh_region_by_name("US");
    CHECK(mesh_num_channels(us, lf) == 104);
    CHECK(mesh_channel_slot(us, lf, "", 0) == 19);
    CHECK(NEAR(mesh_frequency_mhz(us, lf, "", 0), 906.875)); /* well-known US LongFast */

    const mesh_region_t *eu = mesh_region_by_name("EU_868");
    CHECK(mesh_num_channels(eu, lf) == 1);
    CHECK(NEAR(mesh_frequency_mhz(eu, lf, "", 0), 869.525)); /* well-known EU LongFast */

    const mesh_region_t *in = mesh_region_by_name("IN");
    CHECK(mesh_num_channels(in, lf) == 8);
    CHECK(NEAR(mesh_frequency_mhz(in, lf, "", 0), 865.875));

    /* Explicit 1-based slot overrides the hash. */
    CHECK(NEAR(mesh_frequency_mhz(us, lf, "", 1), 902.125));
    CHECK(mesh_region_by_code(10) == in);
    CHECK(mesh_region_by_code(0) == NULL);

    /* Meshtastic 2.7 additions. Expected values come from Meshtastic's formula in float32, worked out apart from
     * this code: slot = djb2(name) % floor(span / bw), freq = start + bw/2 + slot * bw. */
    const mesh_preset_t *lt = mesh_preset_by_name("LONG_TURBO");
    CHECK(lt && lt->code == 9 && lt->bw_khz == 500.0f && lt->sf == 11 && lt->cr == 8 && strcmp(lt->display, "LongTurbo") == 0);
    CHECK(mesh_preset_by_code(9) == lt && mesh_preset_by_name("LongTurbo") == lt);
    CHECK(mesh_djb2("LongTurbo") == 26541281u);
    CHECK(mesh_num_channels(us, lt) == 52 && mesh_channel_slot(us, lt, "", 0) == 13);
    CHECK(NEAR(mesh_frequency_mhz(us, lt, "", 0), 908.75));

    static const struct { const char *name; uint8_t code; uint32_t n, slot; double mhz; } NEW[] = {
        {"PH_433", 19, 6, 5, 434.375},   {"PH_868", 20, 5, 0, 868.125},   {"PH_915", 21, 12, 11, 917.875},
        {"ANZ_433", 22, 6, 5, 434.425},  {"KZ_433", 23, 6, 5, 434.45},    {"KZ_863", 24, 20, 15, 866.875},
        {"NP_865", 25, 12, 11, 867.875}, {"BR_902", 26, 22, 7, 903.875},
    };
    for (size_t i = 0; i < sizeof(NEW) / sizeof(NEW[0]); i++) {
        const mesh_region_t *r = mesh_region_by_name(NEW[i].name);
        CHECK(r && r->code == NEW[i].code && mesh_region_by_code(NEW[i].code) == r);
        if (!r) continue;
        CHECK(mesh_num_channels(r, lf) == NEW[i].n);
        CHECK(mesh_channel_slot(r, lf, "", 0) == NEW[i].slot);
        CHECK(NEAR(mesh_frequency_mhz(r, lf, "", 0), NEW[i].mhz));
    }
    const mesh_region_t *th = mesh_region_by_name("TH");
    CHECK(th && th->duty_cycle == 10 && th->power_limit == 27); /* NBTC rules, as in Meshtastic 2.7 */
    CHECK(mesh_region_by_code(13) == NULL); /* LORA_24 needs a 2.4 GHz radio */

    /* A preset wider than the band falls back to LONG_FAST; a band exactly as wide still fits. */
    const mesh_preset_t *st = mesh_preset_by_name("SHORT_TURBO");
    CHECK(mesh_preset_fits(eu, lf));
    CHECK(!mesh_preset_fits(eu, st) && !mesh_preset_fits(eu, lt));
    CHECK(mesh_preset_fits(mesh_region_by_name("RU"), lt));     /* 0.5 MHz band */
    CHECK(mesh_preset_fits(mesh_region_by_name("UA_868"), lt)); /* 0.6 MHz band */
    CHECK(mesh_preset_fits(us, lt));
}

static void test_channel_hash(void)
{
    uint8_t key[32];
    uint8_t one = 1;
    size_t kl = mesh_expand_psk(&one, 1, key);
    CHECK(kl == 16);
    CHECK(key[0] == 0xd4 && key[15] == 0x01);
    CHECK(mesh_channel_hash("LongFast", key, kl) == 8); /* default LongFast channel hash */

    uint8_t two = 2;
    kl = mesh_expand_psk(&two, 1, key);
    CHECK(kl == 16 && key[15] == 0x02);

    uint8_t zero = 0;
    CHECK(mesh_expand_psk(&zero, 1, key) == 0);
    CHECK(mesh_expand_psk(NULL, 0, key) == 0);

    uint8_t k32[32];
    memset(k32, 0xab, sizeof(k32));
    CHECK(mesh_expand_psk(k32, 32, key) == 32 && key[31] == 0xab);
}

static void test_airtime(void)
{
    const mesh_preset_t *lf = mesh_preset_by_name("LONG_FAST");
    CHECK(NEAR(mesh_symbol_ms(lf), 8.192));
    uint32_t t = mesh_airtime_ms(lf, 50);
    CHECK(t > 500 && t < 800); /* ~ 620 ms for 50 bytes at SF11/250k */
    CHECK(mesh_airtime_ms(lf, 255) > mesh_airtime_ms(lf, 50));
    const mesh_preset_t *ls = mesh_preset_by_name("LONG_SLOW");
    CHECK(mesh_airtime_ms(ls, 50) > 2 * t);
}

static void test_header(void)
{
    mt_header_t h = {.to = MT_BROADCAST, .from = 0x12345678, .id = 0xdeadbeef,
                     .flags = mt_make_flags(3, 3, true), .channel = 8, .next_hop = 0, .relay_node = 0x78};
    uint8_t b[16];
    mt_header_write(&h, b);
    CHECK(b[0] == 0xff && b[4] == 0x78 && b[7] == 0x12 && b[8] == 0xef && b[11] == 0xde);
    CHECK(b[12] == 0x6b); /* hop_limit 3 | want_ack | hop_start 3<<5 */
    mt_header_t r;
    mt_header_read(b, &r);
    CHECK(r.to == h.to && r.from == h.from && r.id == h.id && r.flags == h.flags && r.relay_node == 0x78);
    CHECK(mt_hop_limit(&r) == 3 && mt_hop_start(&r) == 3 && mt_want_ack(&r));
}

static void test_data_roundtrip(void)
{
    const char *text = "hello mesh";
    mt_data_t d = {.portnum = MT_PORT_TEXT, .payload = (const uint8_t *)text, .payload_len = strlen(text),
                   .has_bitfield = true, .bitfield = 0};
    uint8_t buf[64];
    size_t n = mt_data_encode(&d, buf, sizeof(buf));
    /* 08 01 | 12 0a "hello mesh" | 48 00 */
    CHECK(n == 2 + 2 + 10 + 2);
    CHECK(buf[0] == 0x08 && buf[1] == 0x01 && buf[2] == 0x12 && buf[3] == 10 && buf[14] == 0x48);
    mt_data_t o;
    CHECK(mt_data_decode(buf, n, &o));
    CHECK(o.portnum == MT_PORT_TEXT && o.payload_len == 10 && memcmp(o.payload, text, 10) == 0);
    CHECK(o.has_bitfield && o.bitfield == 0);

    mt_data_t ack = {.portnum = MT_PORT_ROUTING, .request_id = 0xcafef00d};
    uint8_t rp[8];
    size_t rl = mt_routing_encode_error(0, rp, sizeof(rp));
    CHECK(rl == 2 && rp[0] == 0x18 && rp[1] == 0x00);
    ack.payload = rp;
    ack.payload_len = rl;
    n = mt_data_encode(&ack, buf, sizeof(buf));
    CHECK(mt_data_decode(buf, n, &o) && o.request_id == 0xcafef00d && o.portnum == MT_PORT_ROUTING);
    mt_routing_t rt;
    CHECK(mt_routing_decode(o.payload, o.payload_len, &rt) && rt.has_error && rt.error_reason == 0);

    /* Noise must not decode as a valid Data message. */
    uint8_t noise[] = {0xff, 0xff, 0xff, 0x01, 0x02};
    CHECK(!mt_data_decode(noise, sizeof(noise), &o));
    uint8_t noport[] = {0x12, 0x01, 0x41};
    CHECK(!mt_data_decode(noport, sizeof(noport), &o));
    /* Overflow is reported. */
    CHECK(mt_data_encode(&d, buf, 5) == 0);
}

static void test_user_position(void)
{
    mt_user_t u = {.hw_model = MT_HW_HELTEC_V3};
    strcpy(u.id, "!a1b2c3d4");
    strcpy(u.long_name, "Sotreus Edge 1A2B");
    strcpy(u.short_name, "SE2B");
    uint8_t buf[96];
    size_t n = mt_user_encode(&u, buf, sizeof(buf));
    CHECK(n > 0);
    mt_user_t o;
    CHECK(mt_user_decode(buf, n, &o));
    CHECK(strcmp(o.id, u.id) == 0 && strcmp(o.long_name, u.long_name) == 0 && strcmp(o.short_name, "SE2B") == 0);
    CHECK(o.hw_model == MT_HW_HELTEC_V3 && !o.has_public_key);

    /* Position: lat 12.9716 (sfixed32 129716000), lon 77.5946, alt -5 (int32 varint, 10 bytes). */
    uint8_t p[] = {0x0d, 0x20, 0x4f, 0xbb, 0x07, 0x15, 0x10, 0xff, 0x3f, 0x2e,
                   0x18, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01};
    mt_position_t pos;
    CHECK(mt_position_decode(p, sizeof(p), &pos));
    CHECK(pos.has_lat && pos.lat_i == 129716000);
    CHECK(pos.has_lon && pos.lon_i == 775946000);
    CHECK(pos.has_alt && pos.alt == -5);
}

static void test_util(void)
{
    uint8_t mac[6];
    CHECK(edge_mac_parse("AA:bb:01:02:03:fe", mac) && mac[0] == 0xaa && mac[5] == 0xfe);
    CHECK(edge_mac_parse("aabb010203fe", mac));
    CHECK(!edge_mac_parse("aa:bb:01:02:03", mac));
    CHECK(!edge_mac_parse("a:abb:01:02:03:fe", mac));
    char s[18];
    edge_mac_str(mac, s);
    CHECK(strcmp(s, "aa:bb:01:02:03:fe") == 0);

    uint8_t out[40];
    CHECK(edge_b64_decode("AQ==", out, sizeof(out)) == 1 && out[0] == 1);
    int n = edge_b64_decode("1PG7OiApB1nwvP+rz05pAQ==", out, sizeof(out));
    CHECK(n == 16 && out[0] == 0xd4 && out[15] == 0x01);
    CHECK(edge_b64_decode("!!", out, sizeof(out)) == -1);
    char enc[40];
    CHECK(edge_b64_encode(out, 16, enc, sizeof(enc)) == 24 && strcmp(enc, "1PG7OiApB1nwvP+rz05pAQ==") == 0);

    char t[16];
    const uint8_t bad[] = {'a', 0x01, 0xc3, 0xa9, 0xff, 'z'};
    edge_sanitize_text(t, sizeof(t), bad, sizeof(bad));
    CHECK(strcmp(t, "a?\xc3\xa9?z") == 0);
    const uint8_t cut[] = {'a', 'b', 0xe2, 0x82, 0xac};
    edge_sanitize_text(t, 5, cut, sizeof(cut)); /* room for "ab" + NUL only */
    CHECK(strcmp(t, "ab") == 0);

    CHECK(edge_fnv1a(NULL, 0, (const uint8_t *)"", 0) == 2166136261u);
    CHECK(edge_fnv1a(NULL, 0, (const uint8_t *)"a", 1) == 0xe40c292cu);

    /* seen.body trims trailing zeros; the phone pads them back. */
    const uint8_t body[] = {1, 0, 2, 0, 0, 0};
    CHECK(edge_trim_zeros(body, sizeof(body)) == 3);
    CHECK(edge_trim_zeros(body, 0) == 0);
    const uint8_t zeros[4] = {0};
    CHECK(edge_trim_zeros(zeros, sizeof(zeros)) == 0);

    /* live rows */
    char row[96];
    const uint8_t lmac[6] = {0xca, 0x2a, 0xf0, 0xe6, 0xb6, 0x01};
    int rn = edge_live_row(row, sizeof(row), "wifi_probe", lmac, -52, -44, 1200, 4);
    CHECK(rn > 0 && strcmp(row, "[\"wifi_probe\",\"ca:2a:f0:e6:b6:01\",-52,-44,1200,4]") == 0);
    CHECK(edge_live_row(row, sizeof(row), "ble", lmac, -60, -50, -5, 1) > 0 && strstr(row, ",0,1]") != NULL);
    CHECK(edge_live_row(row, 10, "ble", lmac, -60, -50, 1, 1) == -1);
}

/* Query for "captive.apple.com" with the given type, plus an EDNS OPT record. */
static int make_query(uint8_t *q, uint16_t qtype)
{
    static const uint8_t hdr[] = {0x12, 0x34, 0x01, 0x00, 0, 1, 0, 0, 0, 0, 0, 1};
    int n = 0;
    memcpy(q, hdr, sizeof(hdr));
    n = sizeof(hdr);
    static const char *labels[] = {"captive", "apple", "com"};
    for (int i = 0; i < 3; i++) {
        q[n++] = (uint8_t)strlen(labels[i]);
        memcpy(q + n, labels[i], strlen(labels[i]));
        n += (int)strlen(labels[i]);
    }
    q[n++] = 0;
    q[n++] = (uint8_t)(qtype >> 8);
    q[n++] = (uint8_t)qtype;
    q[n++] = 0;
    q[n++] = 1;
    static const uint8_t opt[] = {0, 0, 41, 0x10, 0, 0, 0, 0, 0, 0, 0};
    memcpy(q + n, opt, sizeof(opt));
    return n + (int)sizeof(opt);
}

static void test_dns(void)
{
    uint8_t q[512];
    int qlen = make_query(q, 1);
    int qend = qlen - 11;
    int r = edge_dns_answer(q, qlen, sizeof(q), false);
    CHECK(r == qend + 16);
    CHECK(q[0] == 0x12 && q[1] == 0x34);          /* id kept */
    CHECK((q[2] & 0x80) && (q[2] & 0x04));        /* response, authoritative */
    CHECK((q[3] & 0x0f) == 0);                    /* NOERROR */
    CHECK(q[7] == 1 && q[11] == 0);               /* one answer, EDNS dropped */
    CHECK(q[r - 4] == 192 && q[r - 3] == 168 && q[r - 2] == 4 && q[r - 1] == 1);
    CHECK(q[qend] == 0xc0 && q[qend + 1] == 0x0c);

    qlen = make_query(q, 28); /* AAAA */
    r = edge_dns_answer(q, qlen, sizeof(q), false);
    CHECK(r == qlen - 11 && q[7] == 0 && (q[3] & 0x0f) == 0);

    qlen = make_query(q, 1);
    r = edge_dns_answer(q, qlen, sizeof(q), true); /* logged-in client */
    CHECK(r == qlen - 11 && q[7] == 0 && (q[3] & 0x0f) == 3);

    qlen = make_query(q, 1);
    q[2] |= 0x80; /* a response: ignore */
    CHECK(edge_dns_answer(q, qlen, sizeof(q), false) == 0);
    qlen = make_query(q, 1);
    CHECK(edge_dns_answer(q, 20, sizeof(q), false) == 0); /* truncated */
    CHECK(edge_dns_answer(q, 5, sizeof(q), false) == 0);
}


static void unhex(const char *h, uint8_t *o) { size_t n = strlen(h) / 2; for (size_t i = 0; i < n; i++) { unsigned v; sscanf(h + 2 * i, "%2x", &v); o[i] = (uint8_t)v; } }
static int eqhex(const uint8_t *b, const char *h) { uint8_t e[256]; size_t n = strlen(h) / 2; unhex(h, e); return memcmp(b, e, n) == 0; }

static void test_crypto(void)
{
    uint8_t out[64], k[32], p[32];
    /* SHA-256 (FIPS 180-2) */
    mesh_sha256((const uint8_t *)"abc", 3, out);
    CHECK(eqhex(out, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    mesh_sha256((const uint8_t *)"", 0, out);
    CHECK(eqhex(out, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
    const char *m56 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    mesh_sha256((const uint8_t *)m56, strlen(m56), out);
    CHECK(eqhex(out, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));

    /* AES (FIPS-197 C.1 / C.3) */
    mesh_aes_t a;
    uint8_t key[32], pt[16];
    unhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", key);
    unhex("00112233445566778899aabbccddeeff", pt);
    mesh_aes_init(&a, key, 16);
    mesh_aes_encrypt_block(&a, pt, out);
    CHECK(eqhex(out, "69c4e0d86a7b0430d8cdb78070b4c55a"));
    mesh_aes_init(&a, key, 32);
    mesh_aes_encrypt_block(&a, pt, out);
    CHECK(eqhex(out, "8ea2b7ca516745bfeafc49904b496089"));

    /* AES-CCM (RFC 3610 packet vector #1: M = 8, L = 2) */
    uint8_t ck[16], nonce[13], aad[8], msg[23], ct[23], tag[8], back[23];
    unhex("c0c1c2c3c4c5c6c7c8c9cacbcccdcecf", ck);
    unhex("00000003020100a0a1a2a3a4a5", nonce);
    for (int i = 0; i < 8; i++) aad[i] = (uint8_t)i;
    for (int i = 0; i < 23; i++) msg[i] = (uint8_t)(8 + i);
    mesh_ccm_encrypt(ck, 16, nonce, 8, aad, 8, msg, 23, ct, tag);
    CHECK(eqhex(ct, "588c979a61c663d2f066d0c2c0f989806d5f6b61dac384"));
    CHECK(eqhex(tag, "17e8d12cfdf926e0"));
    CHECK(mesh_ccm_decrypt(ck, 16, nonce, 8, aad, 8, ct, 23, tag, back) && memcmp(back, msg, 23) == 0);
    tag[0] ^= 1;
    CHECK(!mesh_ccm_decrypt(ck, 16, nonce, 8, aad, 8, ct, 23, tag, back));

    /* X25519 (RFC 7748 §5.2 and §6.1) */
    unhex("a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4", k);
    unhex("e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c", p);
    mesh_x25519(out, k, p);
    CHECK(eqhex(out, "c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552"));
    uint8_t ap[32], bp[32], apub[32], bpub[32], s1[32], s2[32];
    unhex("77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a", ap);
    unhex("5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb", bp);
    mesh_x25519_public(apub, ap);
    mesh_x25519_public(bpub, bp);
    CHECK(eqhex(apub, "8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a"));
    CHECK(eqhex(bpub, "de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f"));
    mesh_x25519(s1, ap, bpub);
    mesh_x25519(s2, bp, apub);
    CHECK(eqhex(s1, "4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742") && memcmp(s1, s2, 32) == 0);

    /* Meshtastic PKI framing round trip: both sides derive the same key. */
    uint8_t ka[32], kb[32], wire[64], plain[40];
    mesh_pki_shared_key(ka, ap, bpub);
    mesh_pki_shared_key(kb, bp, apub);
    CHECK(memcmp(ka, kb, 32) == 0);
    const char *hello = "direct hello";
    mesh_pki_encrypt(ka, 0x11223344, 0xa1b2c3d4, 0x55667788, (const uint8_t *)hello, strlen(hello), wire);
    size_t wl = strlen(hello) + MESH_PKI_OVERHEAD;
    CHECK(wire[wl - 4] == 0x88 && wire[wl - 1] == 0x55); /* extra nonce trailer, little-endian */
    CHECK(mesh_pki_decrypt(kb, 0x11223344, 0xa1b2c3d4, wire, wl, plain) && memcmp(plain, hello, strlen(hello)) == 0);
    CHECK(!mesh_pki_decrypt(kb, 0x11223345, 0xa1b2c3d4, wire, wl, plain)); /* wrong sender -> wrong nonce */
}

static void test_pb_more(void)
{
    /* Default Meshtastic channel link: https://meshtastic.org/e/#CgMSAQESBggBQANIAQ */
    uint8_t raw[64], enc[64];
    int n = edge_b64_decode("CgMSAQESBggBQANIAQ", raw, sizeof(raw));
    CHECK(n == 13);
    mt_channelset_t cs;
    CHECK(mt_channelset_decode(raw, (size_t)n, &cs));
    CHECK(cs.n == 1 && cs.ch[0].psk_len == 1 && cs.ch[0].psk[0] == 1 && cs.ch[0].name[0] == 0);
    CHECK(cs.has_lora && cs.use_preset && cs.hop_limit == 3 && cs.tx_enabled && cs.region == 0);
    size_t el = mt_channelset_encode(&cs, enc, sizeof(enc));
    CHECK(el == (size_t)n && memcmp(enc, raw, el) == 0);

    /* Traceroute: packed fields round trip, negative SNR kept. */
    mt_route_t rt = {0}, ro;
    rt.n_route = 2; rt.route[0] = 0x11111111; rt.route[1] = 0x22222222;
    rt.n_snr_towards = 3; rt.snr_towards[0] = 20; rt.snr_towards[1] = -12; rt.snr_towards[2] = 7;
    uint8_t b[96];
    size_t bl = mt_route_encode(&rt, b, sizeof(b));
    CHECK(bl > 0 && mt_route_decode(b, bl, &ro));
    CHECK(ro.n_route == 2 && ro.route[1] == 0x22222222 && ro.n_snr_towards == 3 && ro.snr_towards[1] == -12 && ro.n_route_back == 0);

    /* Telemetry device metrics. */
    mt_telemetry_t t = {.time = 1700000000, .has_battery = true, .battery_level = 101, .has_voltage = true, .voltage = 4.2f,
                        .has_ch_util = true, .channel_utilization = 12.5f, .has_air_util = true, .air_util_tx = 0.75f,
                        .has_uptime = true, .uptime_seconds = 3600}, to;
    bl = mt_telemetry_encode(&t, b, sizeof(b));
    CHECK(bl > 0 && mt_telemetry_decode(b, bl, &to) && to.has_device && to.battery_level == 101 && to.uptime_seconds == 3600);
    CHECK(to.voltage > 4.19f && to.voltage < 4.21f && to.channel_utilization == 12.5f && to.time == 1700000000);

    /* Position precision: 13 bits keeps ~2.9 km cells, centred. */
    int32_t lat = 129716000;
    int32_t tr = mt_position_truncate(lat, 13);
    CHECK(tr != lat && (uint32_t)(tr - lat) < (1u << 19) + (1u << 18));
    CHECK(mt_position_truncate(lat, 32) == lat);
    mt_position_t pos = {.has_lat = true, .has_lon = true, .lat_i = tr, .lon_i = -1223000000, .has_alt = true, .alt = -3,
                         .location_source = 1, .precision_bits = 13}, po;
    bl = mt_position_encode(&pos, b, sizeof(b));
    CHECK(bl > 0 && mt_position_decode(b, bl, &po) && po.lat_i == tr && po.lon_i == -1223000000 && po.alt == -3 && po.precision_bits == 13);

    /* User with public key. */
    mt_user_t u = {.hw_model = 43, .has_public_key = true}, uo;
    strcpy(u.id, "!00000001");
    for (int i = 0; i < 32; i++) u.public_key[i] = (uint8_t)i;
    bl = mt_user_encode(&u, b, sizeof(b));
    CHECK(bl > 0 && mt_user_decode(b, bl, &uo) && uo.has_public_key && uo.public_key[31] == 31);

    /* Waypoint, neighbor info, paxcount, store & forward. */
    mt_waypoint_t w = {.id = 7, .has_lat = true, .has_lon = true, .lat_i = 1, .lon_i = -2, .icon = 0x1F3E0}, wo;
    strcpy(w.name, "Camp");
    bl = mt_waypoint_encode(&w, b, sizeof(b));
    CHECK(bl > 0 && mt_waypoint_decode(b, bl, &wo) && wo.id == 7 && wo.lon_i == -2 && strcmp(wo.name, "Camp") == 0 && wo.icon == 0x1F3E0);
    mt_neighborinfo_t ni = {.node_id = 5, .interval_secs = 900, .n = 2}, nio;
    ni.nb[0].node_id = 9; ni.nb[0].snr = -6.25f; ni.nb[1].node_id = 10; ni.nb[1].snr = 3.0f;
    bl = mt_neighborinfo_encode(&ni, b, sizeof(b));
    CHECK(bl > 0 && mt_neighborinfo_decode(b, bl, &nio) && nio.n == 2 && nio.nb[0].snr == -6.25f && nio.interval_secs == 900);
    uint32_t pw, pb2, pu;
    bl = mt_paxcount_encode(12, 34, 56, b, sizeof(b));
    CHECK(mt_paxcount_decode(b, bl, &pw, &pb2, &pu) && pw == 12 && pb2 == 34 && pu == 56);
    mt_sf_t sf = {.rr = MT_SF_CLIENT_HISTORY, .has_history = true, .window = 120}, sfo;
    bl = mt_sf_encode(&sf, b, sizeof(b));
    CHECK(bl > 0 && mt_sf_decode(b, bl, &sfo) && sfo.rr == MT_SF_CLIENT_HISTORY && sfo.has_history && sfo.window == 120);
}

/* Finds field `f` at the top level of a message; returns its sub-buffer or value. */
static bool find_field(const uint8_t *b, size_t n, uint32_t f, pf_t *out)
{
    pr_t r;
    pr_init(&r, b, n);
    while (pr_next(&r, out) == 1) {
        if (out->field == f) return true;
    }
    return false;
}

static void base_cfg(edge_config_t *c, edge_mesh_mod_t *m)
{
    memset(c, 0, sizeof(*c));
    c->region = 10; /* IN */
    c->preset = 0;  /* LONG_FAST */
    c->hop_limit = 3;
    c->n_channels = 1;
    c->channels[0].psk[0] = 1;
    c->channels[0].psk_len = 1;
    c->channels[0].position_precision = 13;
    c->nodeinfo_interval_s = 10800;
    strcpy(c->long_name, "Edge");
    strcpy(c->short_name, "ED");
    c->mx.tx_enabled = true;
    c->mx.n_canned = 2;
    strcpy(c->mx.canned[0], "OK");
    strcpy(c->mx.canned[1], "On my way");
    memset(m, 0, sizeof(*m));
    m->admin_enabled = true;
    strcpy(m->mqtt_address, "mqtt.meshtastic.org");
    strcpy(m->mqtt_password, "secret");
}

static void test_admin(void)
{
    edge_config_t c;
    edge_mesh_mod_t m;
    base_cfg(&c, &m);
    uint8_t pk[8] = {1, 2, 3, 4, 5, 6, 7, 8}, pub[32];
    memset(pub, 0xab, sizeof(pub));
    adm_ctx_t x = {.my_node = 0x11223344, .public_key = pub, .fw_version = "2.5.0.5e0e", .passkey = pk};
    adm_class_t cls;
    adm_result_t res;
    uint8_t req[64];
    pf_t f, g, h;

    /* get_config LORA_CONFIG (5): a read, needs no passkey */
    size_t rl = mesh_admin_build_get_config(5, NULL, req, sizeof(req));
    CHECK(mesh_admin_classify(req, rl, &cls) && cls.variant == 5 && cls.is_get && !cls.sensitive);
    CHECK(mesh_admin_apply(req, rl, &x, &c, &m, &res) && res.reply_len > 0);
    CHECK(find_field(res.reply, res.reply_len, 101, &f) && f.len == 8 && memcmp(f.data, pk, 8) == 0);
    CHECK(find_field(res.reply, res.reply_len, 6, &f));                   /* get_config_response */
    CHECK(find_field(f.data, f.len, 6, &g));                               /* Config.lora */
    CHECK(find_field(g.data, g.len, 7, &h) && h.v == 10);                  /* region IN */
    CHECK(find_field(g.data, g.len, 8, &h) && h.v == 3);                   /* hop limit */

    /* get_module_config MQTT without admin-key rights: no password */
    rl = mesh_admin_build_get_module(0, NULL, req, sizeof(req));
    CHECK(mesh_admin_apply(req, rl, &x, &c, &m, &res) && find_field(res.reply, res.reply_len, 8, &f));
    CHECK(find_field(f.data, f.len, 1, &g) && find_field(g.data, g.len, 2, &h) && h.len == 19);
    CHECK(!find_field(g.data, g.len, 4, &h));
    x.sensitive_ok = true;
    CHECK(mesh_admin_apply(req, rl, &x, &c, &m, &res) && find_field(res.reply, res.reply_len, 8, &f));
    CHECK(find_field(f.data, f.len, 1, &g) && find_field(g.data, g.len, 4, &h) && h.len == 6);
    x.sensitive_ok = false;

    /* get_channel 0 (sent as index + 1): no key unless sensitive_ok */
    rl = mesh_admin_build_get_channel(0, req, sizeof(req));
    CHECK(mesh_admin_apply(req, rl, &x, &c, &m, &res) && find_field(res.reply, res.reply_len, 2, &f));
    CHECK(find_field(f.data, f.len, 3, &g) && g.v == 1);                   /* PRIMARY */
    CHECK(find_field(f.data, f.len, 2, &g) && !find_field(g.data, g.len, 2, &h));

    /* get canned messages */
    pw_t w;
    pw_init(&w, req, sizeof(req));
    pw_u(&w, 10, 1);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && find_field(res.reply, res.reply_len, 11, &f));
    CHECK(f.len == 12 && memcmp(f.data, "OK|On my way", 12) == 0);

    /* set_config lora: hop 5, region EU_868 (3), with a passkey; classified as a non-sensitive write */
    pw_init(&w, req, sizeof(req));
    pw_mark_t a = pw_begin(&w, 34);
    pw_mark_t l = pw_begin(&w, 6);
    pw_bool(&w, 1, true);
    pw_u(&w, 7, 3);
    pw_u(&w, 8, 5);
    pw_bool(&w, 9, true);
    pw_end(&w, l);
    pw_end(&w, a);
    pw_bytes(&w, 101, pk, 8);
    CHECK(mesh_admin_classify(req, w.len, &cls) && cls.variant == 34 && !cls.is_get && !cls.sensitive && cls.has_passkey);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && res.cfg_changed && c.region == 3 && c.hop_limit == 5);

    /* set_config device: role CLIENT_BASE (12) is taken; 13 is not a role and leaves it unchanged */
    for (uint32_t role = 12; role <= 13; role++) {
        pw_init(&w, req, sizeof(req));
        a = pw_begin(&w, 34);
        l = pw_begin(&w, 1);
        pw_u(&w, 1, role);
        pw_end(&w, l);
        pw_end(&w, a);
        pw_bytes(&w, 101, pk, 8);
        CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && c.mx.role == 12);
    }

    /* set_config lora: LONG_TURBO (9) and region BR_902 (26) are taken */
    pw_init(&w, req, sizeof(req));
    a = pw_begin(&w, 34);
    l = pw_begin(&w, 6);
    pw_bool(&w, 1, true);
    pw_u(&w, 2, 9);
    pw_u(&w, 7, 26);
    pw_end(&w, l);
    pw_end(&w, a);
    pw_bytes(&w, 101, pk, 8);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && c.preset == 9 && c.region == 26);

    /* set_config security is sensitive; listing a key switches to keys-only */
    uint8_t key[32];
    memset(key, 7, sizeof(key));
    pw_init(&w, req, sizeof(req));
    a = pw_begin(&w, 34);
    l = pw_begin(&w, 8);
    pw_bytes(&w, 3, key, 32);
    pw_end(&w, l);
    pw_end(&w, a);
    CHECK(mesh_admin_classify(req, w.len, &cls) && cls.sensitive);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && res.mod_changed && m.n_admin_keys == 1 &&
          m.admin_mode == EDGE_ADMIN_KEYS && m.admin_keys[0][5] == 7);

    /* set_module_config MQTT is sensitive */
    pw_init(&w, req, sizeof(req));
    a = pw_begin(&w, 35);
    l = pw_begin(&w, 1);
    pw_bool(&w, 1, true);
    pw_str(&w, 2, "broker.example:1884");
    pw_bool(&w, 7, true);
    pw_end(&w, l);
    pw_end(&w, a);
    CHECK(mesh_admin_classify(req, w.len, &cls) && cls.sensitive);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && m.mqtt_enabled && m.mqtt_tls &&
          strcmp(m.mqtt_address, "broker.example:1884") == 0);

    /* set_channel: add a secondary with downlink, then disable it */
    pw_init(&w, req, sizeof(req));
    a = pw_begin(&w, 33);
    pw_u(&w, 1, 1);
    l = pw_begin(&w, 2);
    pw_bytes(&w, 2, key, 16);
    pw_str(&w, 3, "Team");
    pw_bool(&w, 6, true);
    pw_end(&w, l);
    pw_u(&w, 3, 2);
    pw_end(&w, a);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && c.n_channels == 2 && strcmp(c.channels[1].name, "Team") == 0 &&
          c.channels[1].psk_len == 16 && (m.ch_downlink & 2) && !(m.ch_uplink & 2));
    pw_init(&w, req, sizeof(req));
    a = pw_begin(&w, 33);
    pw_u(&w, 1, 1);
    pw_end(&w, a);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && c.n_channels == 1 && !(m.ch_downlink & 2));

    /* canned messages, reboot, favourite */
    pw_init(&w, req, sizeof(req));
    pw_str(&w, 36, "A|B|C");
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && c.mx.n_canned == 3 && strcmp(c.mx.canned[2], "C") == 0);
    pw_init(&w, req, sizeof(req));
    pw_u(&w, 97, 5);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && res.reboot_s == 5);
    pw_init(&w, req, sizeof(req));
    pw_u(&w, 39, 0xdeadbeef);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && res.fav_set == 0xdeadbeef);

    /* metadata */
    pw_init(&w, req, sizeof(req));
    pw_u(&w, 12, 1);
    CHECK(mesh_admin_apply(req, w.len, &x, &c, &m, &res) && find_field(res.reply, res.reply_len, 13, &f));
    CHECK(find_field(f.data, f.len, 9, &g) && g.v == 43 && find_field(f.data, f.len, 11, &g) && g.v == 1);
    CHECK(mesh_admin_baud_value(mesh_admin_baud_code(9600)) == 9600);
}

static void test_ext_pb(void)
{
    uint8_t payload[4] = {9, 8, 7, 6}, b[256];
    mt_envelope_t e = {.has_packet = true}, eo;
    e.packet = (mt_packet_t){.from = 0x11, .to = 0xffffffff, .id = 0x55667788, .channel = 8, .payload = payload,
                             .payload_len = 4, .hop_limit = 3, .hop_start = 3, .rx_rssi = -101, .rx_snr = -7.5f, .via_mqtt = true};
    strcpy(e.channel_id, "LongFast");
    strcpy(e.gateway_id, "!11223344");
    size_t n = mt_envelope_encode(&e, b, sizeof(b));
    CHECK(n > 0 && mt_envelope_decode(b, n, &eo));
    CHECK(eo.packet.from == 0x11 && eo.packet.to == 0xffffffff && eo.packet.id == 0x55667788 && eo.packet.channel == 8);
    CHECK(!eo.packet.decoded && eo.packet.payload_len == 4 && memcmp(eo.packet.payload, payload, 4) == 0);
    CHECK(eo.packet.rx_rssi == -101 && eo.packet.rx_snr == -7.5f && eo.packet.via_mqtt && eo.packet.hop_start == 3);
    CHECK(strcmp(eo.channel_id, "LongFast") == 0 && strcmp(eo.gateway_id, "!11223344") == 0);

    mt_hw_t h = {.type = MT_HW_READ_GPIOS_REPLY, .mask = 1ULL << 40 | 1 << 4, .value = 1ULL << 40}, ho;
    n = mt_hw_encode(&h, b, sizeof(b));
    CHECK(mt_hw_decode(b, n, &ho) && ho.type == 5 && ho.mask == h.mask && ho.value == h.value);

    mt_sf_t sf;
    n = mt_sf_encode_history(MT_SF_ROUTER_HISTORY, 7, 240 * 60000, 0, b, sizeof(b));
    CHECK(mt_sf_decode(b, n, &sf) && sf.rr == MT_SF_ROUTER_HISTORY && sf.has_history && sf.history_messages == 7);
    n = mt_sf_encode_text(MT_SF_ROUTER_TEXT_BROADCAST, (const uint8_t *)"hi", 2, b, sizeof(b));
    CHECK(mt_sf_decode(b, n, &sf) && sf.rr == MT_SF_ROUTER_TEXT_BROADCAST && sf.text_len == 2);
    n = mt_sf_encode_heartbeat(900, 0, b, sizeof(b));
    CHECK(mt_sf_decode(b, n, &sf) && sf.rr == MT_SF_ROUTER_HEARTBEAT && sf.has_heartbeat && sf.hb_period == 900);
    mt_sf_stats_t st = {.messages_saved = 3, .messages_max = 1008, .heartbeat = true};
    n = mt_sf_encode_stats(&st, b, sizeof(b));
    pf_t f, g;
    CHECK(find_field(b, n, 2, &f) && find_field(f.data, f.len, 3, &g) && g.v == 1008);
    n = mt_sf_encode_rr(MT_SF_CLIENT_PING, b, sizeof(b));
    CHECK(mt_sf_decode(b, n, &sf) && sf.rr == MT_SF_CLIENT_PING);

    mt_mapreport_t mr = {.long_name = "Edge", .short_name = "ED", .firmware_version = "2.5.0.5e0e", .hw_model = 43,
                         .region = 10, .has_position = true, .lat_i = 129000000, .lon_i = 775000000, .position_precision = 14};
    n = mt_mapreport_encode(&mr, b, sizeof(b));
    CHECK(n > 0 && find_field(b, n, 9, &f) && (int32_t)f.v == 129000000 && find_field(b, n, 12, &f) && f.v == 14);

    /* pw_begin/pw_end with a long (2-byte length) body */
    uint8_t big[300];
    memset(big, 'x', sizeof(big));
    pw_t w;
    pw_init(&w, b, sizeof(b));
    pw_mark_t m = pw_begin(&w, 1);
    pw_bytes(&w, 2, big, 200);
    pw_end(&w, m);
    CHECK(!w.overflow && find_field(b, w.len, 1, &f) && f.len == 203 && find_field(f.data, f.len, 2, &g) && g.len == 200);
}

static void test_tak(void)
{
    uint8_t b[256];
    mt_tak_t t = {.cot_type = MT_COT_A_F_G_U_C, .how = MT_COT_HOW_H_E, .team = 10, .role = 1, .has_pos = true,
                  .lat_i = 129716000, .lon_i = 775946000, .alt = -12, .speed = 3, .course = 9000, .battery = 77,
                  .geo_src = MT_GEO_SRC_USER, .alt_src = MT_GEO_SRC_USER}, o;
    strcpy(t.callsign, "EDGE-1");
    strcpy(t.uid, "!11223344");
    b[0] = MT_TAK_RAW_FLAGS;
    size_t n = mt_takv2_encode(&t, b + 1, sizeof(b) - 1);
    CHECK(n > 0 && mt_tak_port78_decode(b, n + 1, &o) && !o.compressed && o.has_pos);
    CHECK(o.lat_i == 129716000 && o.lon_i == 775946000 && o.alt == -12 && o.course == 9000 && o.battery == 77);
    CHECK(o.team == 10 && o.role == 1 && strcmp(o.callsign, "EDGE-1") == 0 && strcmp(o.uid, "!11223344") == 0);
    pf_t f;
    CHECK(find_field(b + 1, n, 8, &f) && f.v == 23); /* altitude is sint32: zigzag(-12) = 23 */
    /* chat */
    mt_tak_t c = {.cot_type = MT_COT_B_T_F, .has_chat = true};
    strcpy(c.callsign, "EDGE-1");
    strcpy(c.chat, "Meet at the gate");
    strcpy(c.chat_to, "All Chat Rooms");
    n = mt_takv2_encode(&c, b, sizeof(b));
    CHECK(mt_takv2_decode(b, n, &o) && o.has_chat && strcmp(o.chat, "Meet at the gate") == 0 && strcmp(o.chat_to, "All Chat Rooms") == 0 &&
          o.cot_type == MT_COT_B_T_F && !o.has_pos);
    /* compressed v2 (zstd flags): flagged, not decoded */
    uint8_t z[4] = {0x01, 0x28, 0xb5, 0x2f};
    CHECK(mt_tak_port78_decode(z, sizeof(z), &o) && o.compressed && !o.has_pos);
    /* v1: position readable, compressed strings hidden */
    mt_tak_t v1 = {.team = 5, .role = 2, .battery = 50, .has_pos = true, .lat_i = -337000000, .lon_i = 1512000000, .alt = 40};
    strcpy(v1.callsign, "ALPHA");
    n = mt_takv1_encode(&v1, b, sizeof(b));
    CHECK(mt_takv1_decode(b, n, &o) && !o.v2 && o.has_pos && o.lat_i == -337000000 && o.alt == 40 && o.team == 5 && o.role == 2 &&
          strcmp(o.callsign, "ALPHA") == 0);
    uint8_t cb[300] = {0x08, 0x01};
    memcpy(cb + 2, b, n);
    CHECK(mt_takv1_decode(cb, n + 2, &o) && o.compressed && o.callsign[0] == 0 && o.has_pos);

    /* admin: TAK module config (ModuleConfigType 15 -> field 16) */
    edge_config_t cfg;
    edge_mesh_mod_t m;
    base_cfg(&cfg, &m);
    m.tak_team = 12;
    m.tak_role = 5;
    uint8_t pk[8] = {0};
    adm_ctx_t x = {.my_node = 1, .fw_version = "2.5.0.5e0e", .passkey = pk};
    adm_result_t res;
    uint8_t req[32];
    size_t rl = mesh_admin_build_get_module(15, NULL, req, sizeof(req));
    pf_t g, h;
    CHECK(mesh_admin_apply(req, rl, &x, &cfg, &m, &res) && find_field(res.reply, res.reply_len, 8, &f) &&
          find_field(f.data, f.len, 16, &g) && find_field(g.data, g.len, 1, &h) && h.v == 12);
    pw_t w;
    pw_init(&w, req, sizeof(req));
    pw_mark_t a = pw_begin(&w, 35);
    pw_mark_t l = pw_begin(&w, 16);
    pw_u(&w, 1, 3);
    pw_u(&w, 2, 2);
    pw_end(&w, l);
    pw_end(&w, a);
    CHECK(mesh_admin_apply(req, w.len, &x, &cfg, &m, &res) && res.mod_changed && m.tak_team == 3 && m.tak_role == 2);
}

int main(void)
{
    test_frequency();
    test_channel_hash();
    test_airtime();
    test_header();
    test_data_roundtrip();
    test_user_position();
    test_util();
    test_dns();
    test_crypto();
    test_pb_more();
    test_admin();
    test_ext_pb();
    test_tak();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
