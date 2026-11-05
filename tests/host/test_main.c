// Host tests for the pure-C parts of the firmware: Meshtastic radio math,
// protobuf codec, and helpers. Run with `make -C tests/host`.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

int main(void)
{
    test_frequency();
    test_channel_hash();
    test_airtime();
    test_header();
    test_data_roundtrip();
    test_user_position();
    test_util();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
