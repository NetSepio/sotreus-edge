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
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
