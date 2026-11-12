#include "mesh_admin.h"

#include <stdio.h>
#include <string.h>

#include "edge_util.h"
#include "mesh_pb.h"
#include "mesh_radio.h"
#include "pb_wire.h"

/* AdminMessage payload fields */
enum {
    A_GET_CHANNEL = 1, A_GET_CHANNEL_RESP = 2, A_GET_OWNER = 3, A_GET_OWNER_RESP = 4, A_GET_CONFIG = 5,
    A_GET_CONFIG_RESP = 6, A_GET_MODULE = 7, A_GET_MODULE_RESP = 8, A_GET_CANNED = 10, A_GET_CANNED_RESP = 11,
    A_GET_METADATA = 12, A_GET_METADATA_RESP = 13, A_GET_RINGTONE = 14, A_GET_RINGTONE_RESP = 15, A_GET_CONN = 16,
    A_GET_CONN_RESP = 17, A_SET_HAM = 18, A_GET_RHW_PINS = 19, A_GET_RHW_PINS_RESP = 20, A_DFU = 21, A_DELETE_FILE = 22,
    A_SET_OWNER = 32, A_SET_CHANNEL = 33, A_SET_CONFIG = 34, A_SET_MODULE = 35, A_SET_CANNED = 36, A_SET_RINGTONE = 37,
    A_REMOVE_NODE = 38, A_SET_FAV = 39, A_REMOVE_FAV = 40, A_SET_FIXED_POS = 41, A_REMOVE_FIXED_POS = 42,
    A_SET_TIME = 43, A_SET_IGNORED = 47, A_REMOVE_IGNORED = 48, A_TOGGLE_MUTED = 49, A_BEGIN_EDIT = 64,
    A_COMMIT_EDIT = 65, A_ADD_CONTACT = 66, A_FACTORY_DEVICE = 94, A_REBOOT = 97, A_SHUTDOWN = 98,
    A_FACTORY_CONFIG = 99, A_NODEDB_RESET = 100, A_PASSKEY = 101,
};

/* Config.payload_variant / ConfigType+1 */
enum { C_DEVICE = 1, C_POSITION, C_POWER, C_NETWORK, C_DISPLAY, C_LORA, C_BLUETOOTH, C_SECURITY, C_SESSIONKEY, C_DEVICEUI };
/* ModuleConfig.payload_variant / ModuleConfigType+1 */
enum { M_MQTT = 1, M_SERIAL, M_EXTNOTIF, M_STOREFWD, M_RANGETEST, M_TELEMETRY, M_CANNED, M_AUDIO, M_RHW, M_NEIGHBOR,
       M_AMBIENT, M_DETECTION, M_PAX, M_STATUSMSG, M_TRAFFIC, M_TAK };

static const uint32_t BAUDS[] = {0, 110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
                                 576000, 921600};

uint32_t mesh_admin_baud_code(uint32_t baud)
{
    for (uint32_t i = 1; i < sizeof(BAUDS) / sizeof(BAUDS[0]); i++) {
        if (BAUDS[i] == baud) return i;
    }
    return 0;
}

uint32_t mesh_admin_baud_value(uint32_t code)
{
    return code && code < sizeof(BAUDS) / sizeof(BAUDS[0]) ? BAUDS[code] : 115200;
}

/* Variant number inside a nested oneof message (first field present). */
static uint32_t inner_variant(const uint8_t *p, size_t n)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    return pr_next(&r, &f) == 1 ? f.field : 0;
}

bool mesh_admin_classify(const uint8_t *in, size_t n, adm_class_t *out)
{
    memset(out, 0, sizeof(*out));
    pr_t r;
    pf_t f;
    pr_init(&r, in, n);
    int k;
    while ((k = pr_next(&r, &f)) == 1) {
        if (f.field == A_PASSKEY && f.wt == 2) {
            out->has_passkey = f.len == 8;
            if (f.len == 8) memcpy(out->passkey, f.data, 8);
            continue;
        }
        if (out->variant) continue;
        out->variant = f.field;
        switch (f.field) {
        case A_GET_CHANNEL: case A_GET_OWNER: case A_GET_CONFIG: case A_GET_MODULE: case A_GET_CANNED:
        case A_GET_METADATA: case A_GET_RINGTONE: case A_GET_CONN: case A_GET_RHW_PINS:
            out->is_get = true;
            break;
        case A_SET_CONFIG: {
            uint32_t v = f.wt == 2 ? inner_variant(f.data, f.len) : 0;
            out->sensitive = v == C_NETWORK || v == C_SECURITY;
            break;
        }
        case A_SET_MODULE: {
            uint32_t v = f.wt == 2 ? inner_variant(f.data, f.len) : 0;
            out->sensitive = v == M_MQTT || v == M_RHW;
            break;
        }
        case A_SET_HAM: case A_SET_CHANNEL: case A_FACTORY_DEVICE: case A_FACTORY_CONFIG: case A_DELETE_FILE:
        case A_DFU:
            out->sensitive = true;
            break;
        default:
            break;
        }
    }
    return k == 0;
}

/* ---- Encoders for the Edge's settings as Meshtastic Config / ModuleConfig ------------- */

static void enc_device(pw_t *w, const edge_config_t *c)
{
    pw_u(w, 1, c->mx.role);
    pw_bool(w, 2, true); /* serial console */
    pw_u(w, 6, c->rebroadcast);
    pw_u(w, 7, c->nodeinfo_interval_s);
}

static void enc_position(pw_t *w, const edge_config_t *c)
{
    pw_u(w, 1, c->mx.pos_interval_s);
    pw_bool(w, 3, c->mx.pos_mode == EDGE_POS_FIXED);
    pw_u(w, 13, 2); /* GpsMode NOT_PRESENT: the Edge has no GNSS */
}

static void enc_network(pw_t *w, const edge_config_t *c, const adm_ctx_t *x)
{
    pw_bool(w, 1, c->link_mode != EDGE_LINK_OFF);
    pw_str(w, 3, x->ssid);
    pw_str(w, 5, "pool.ntp.org");
}

static void enc_lora(pw_t *w, const edge_config_t *c)
{
    pw_bool(w, 1, true);
    pw_u(w, 2, c->preset);
    pw_float(w, 6, (float)c->mx.freq_offset_hz);
    pw_u(w, 7, c->region);
    pw_u(w, 8, c->hop_limit);
    pw_bool(w, 9, c->mx.tx_enabled);
    pw_i32(w, 10, c->tx_power);
    pw_u(w, 11, c->channel_num);
    pw_bool(w, 12, c->mx.override_duty_cycle);
    pw_bool(w, 13, c->mx.rx_boosted);
    pw_float(w, 14, c->mx.override_freq_khz / 1000.0f);
    if (c->mx.n_ignore) {
        pw_tag(w, 103, 2);
        size_t len = 0;
        uint8_t tmp[EDGE_IGNORE_MAX * 5];
        pw_t t;
        pw_init(&t, tmp, sizeof(tmp));
        for (int i = 0; i < c->mx.n_ignore; i++) pw_raw_varint(&t, c->mx.ignore[i]);
        len = t.len;
        pw_raw_varint(w, len);
        for (size_t i = 0; i < len; i++) pw_put(w, tmp[i]);
    }
    pw_bool(w, 104, c->mx.ignore_mqtt);
    pw_bool(w, 105, c->mx.ok_to_mqtt);
}

static void enc_security(pw_t *w, const edge_mesh_mod_t *m, const adm_ctx_t *x)
{
    if (x->public_key) pw_bytes(w, 1, x->public_key, 32);
    /* The private key never leaves the Edge. */
    for (int i = 0; i < m->n_admin_keys; i++) pw_bytes(w, 3, m->admin_keys[i], 32);
    pw_bool(w, 5, true);
    pw_bool(w, 8, m->admin_channel);
}

static bool enc_config(pw_t *w, uint32_t type, const edge_config_t *c, const edge_mesh_mod_t *m, const adm_ctx_t *x)
{
    uint32_t field = type + 1;
    pw_mark_t mk = pw_begin(w, field);
    switch (field) {
    case C_DEVICE: enc_device(w, c); break;
    case C_POSITION: enc_position(w, c); break;
    case C_POWER: break;
    case C_NETWORK: enc_network(w, c, x); break;
    case C_DISPLAY: pw_u(w, 1, c->display_timeout_s); break;
    case C_LORA: enc_lora(w, c); break;
    case C_BLUETOOTH: pw_bool(w, 1, true); pw_u(w, 2, 0); break; /* RANDOM_PIN: the code is on the display */
    case C_SECURITY: enc_security(w, m, x); break;
    case C_SESSIONKEY: break;
    default: return false;
    }
    pw_end(w, mk);
    return true;
}

static uint32_t sf_records_or(const adm_ctx_t *x)
{
    return x ? x->sf_capacity : 0;
}

static bool enc_module(pw_t *w, uint32_t type, const edge_config_t *c, const edge_mesh_mod_t *m, const adm_ctx_t *x)
{
    uint32_t field = type + 1;
    const edge_mesh_ext_t *e = &c->mx;
    pw_mark_t mk = pw_begin(w, field);
    switch (field) {
    case M_MQTT: {
        pw_bool(w, 1, m->mqtt_enabled);
        pw_str(w, 2, m->mqtt_address);
        pw_str(w, 3, m->mqtt_username);
        if (x->sensitive_ok) pw_str(w, 4, m->mqtt_password);
        pw_bool(w, 5, m->mqtt_encryption);
        pw_bool(w, 6, m->mqtt_json);
        pw_bool(w, 7, m->mqtt_tls);
        pw_str(w, 8, m->mqtt_root);
        pw_bool(w, 9, m->mqtt_proxy_to_client);
        pw_bool(w, 10, m->mqtt_map);
        pw_mark_t ms = pw_begin(w, 11);
        pw_u(w, 1, m->mqtt_map_interval_s);
        pw_u(w, 2, m->mqtt_map_precision);
        pw_bool(w, 3, m->mqtt_map_location);
        pw_end(w, ms);
        break;
    }
    case M_SERIAL:
        pw_bool(w, 1, e->serial_enabled);
        pw_u(w, 3, e->serial_rx >= 0 ? (uint32_t)e->serial_rx : 0);
        pw_u(w, 4, e->serial_tx >= 0 ? (uint32_t)e->serial_tx : 0);
        pw_u(w, 5, mesh_admin_baud_code(e->serial_baud));
        pw_u(w, 7, 3); /* TEXTMSG */
        break;
    case M_EXTNOTIF:
        pw_bool(w, 1, e->notify_mode != 0);
        pw_u(w, 2, 1000);
        pw_u(w, 3, e->notify_gpio >= 0 ? (uint32_t)e->notify_gpio : 0);
        pw_bool(w, 4, true);
        pw_bool(w, 5, e->notify_mode == 1);
        pw_bool(w, 6, e->notify_mode != 0);
        break;
    case M_STOREFWD:
        pw_bool(w, 1, m->sf_enabled);
        pw_bool(w, 2, m->sf_heartbeat);
        pw_u(w, 3, sf_records_or(x));
        pw_u(w, 4, m->sf_return_max);
        pw_u(w, 5, m->sf_return_window_min);
        pw_bool(w, 6, m->sf_enabled);
        break;
    case M_RANGETEST:
        pw_bool(w, 1, e->range_test_interval_s != 0);
        pw_u(w, 2, e->range_test_interval_s);
        break;
    case M_TELEMETRY:
        pw_u(w, 1, e->telemetry_interval_s);
        pw_bool(w, 14, e->telemetry_interval_s != 0);
        break;
    case M_CANNED:
        pw_bool(w, 9, e->n_canned != 0);
        break;
    case M_AUDIO:
        pw_bool(w, 1, m->audio_enabled);
        pw_u(w, 2, m->audio_ptt >= 0 ? (uint32_t)m->audio_ptt : 0);
        pw_u(w, 3, m->audio_bitrate);
        pw_u(w, 4, m->i2s_ws >= 0 ? (uint32_t)m->i2s_ws : 0);
        pw_u(w, 5, m->i2s_sd >= 0 ? (uint32_t)m->i2s_sd : 0);
        pw_u(w, 6, m->i2s_din >= 0 ? (uint32_t)m->i2s_din : 0);
        pw_u(w, 7, m->i2s_sck >= 0 ? (uint32_t)m->i2s_sck : 0);
        break;
    case M_RHW:
        pw_bool(w, 1, m->rhw_enabled);
        pw_bool(w, 2, m->rhw_undefined_ok);
        for (int i = 0; i < m->n_pins; i++) {
            pw_mark_t mp = pw_begin(w, 3);
            pw_u(w, 1, m->pins[i].gpio);
            pw_str(w, 2, m->pins[i].name);
            pw_u(w, 3, m->pins[i].type);
            pw_end(w, mp);
        }
        break;
    case M_NEIGHBOR:
        pw_bool(w, 1, e->neighbor_info);
        pw_u(w, 2, e->neighbor_interval_s);
        pw_bool(w, 3, true);
        break;
    case M_AMBIENT:
        break;
    case M_DETECTION:
        pw_bool(w, 1, e->detect_enabled);
        pw_u(w, 2, e->detect_min_interval_s);
        pw_str(w, 5, e->detect_name);
        pw_u(w, 6, e->detect_gpio >= 0 ? (uint32_t)e->detect_gpio : 0);
        pw_u(w, 7, e->detect_active_high ? 1 : 0);
        break;
    case M_PAX:
        pw_bool(w, 1, e->pax_enabled);
        pw_u(w, 2, e->pax_interval_s);
        break;
    case M_TAK:
        pw_u(w, 1, m->tak_team);
        pw_u(w, 2, m->tak_role);
        break;
    case M_STATUSMSG: case M_TRAFFIC:
        break;
    default:
        return false;
    }
    pw_end(w, mk);
    return true;
}

static void enc_channel(pw_t *w, uint32_t index, const edge_config_t *c, const edge_mesh_mod_t *m, bool keys)
{
    pw_u(w, 1, index);
    if (index < c->n_channels) {
        const edge_mesh_channel_t *ch = &c->channels[index];
        pw_mark_t ms = pw_begin(w, 2);
        if (keys && ch->psk_len) pw_bytes(w, 2, ch->psk, ch->psk_len);
        pw_str(w, 3, ch->name);
        pw_bool(w, 5, (m->ch_uplink >> index) & 1);
        pw_bool(w, 6, (m->ch_downlink >> index) & 1);
        pw_mark_t mm = pw_begin(w, 7);
        pw_u(w, 1, ch->position_precision);
        pw_end(w, mm);
        pw_end(w, ms);
        pw_u(w, 3, index == 0 ? 1 : 2);
    } else {
        pw_u(w, 3, 0); /* DISABLED */
    }
}

static void enc_owner(pw_t *w, const edge_config_t *c, const adm_ctx_t *x)
{
    mt_user_t u = {.hw_model = MT_HW_HELTEC_V3, .role = c->mx.role, .is_licensed = c->mx.licensed};
    snprintf(u.id, sizeof(u.id), "!%08lx", (unsigned long)x->my_node);
    edge_strlcpy(u.long_name, c->long_name, sizeof(u.long_name));
    edge_strlcpy(u.short_name, c->short_name, sizeof(u.short_name));
    if (x->public_key && !c->mx.licensed) {
        u.has_public_key = true;
        memcpy(u.public_key, x->public_key, 32);
    }
    uint8_t ub[128];
    size_t ul = mt_user_encode(&u, ub, sizeof(ub));
    pw_bytes(w, A_GET_OWNER_RESP, ub, ul);
}

static void enc_metadata(pw_t *w, const edge_config_t *c, const edge_mesh_mod_t *m, const adm_ctx_t *x)
{
    pw_mark_t mk = pw_begin(w, A_GET_METADATA_RESP);
    pw_str(w, 1, x->fw_version);
    pw_u(w, 2, 23);
    pw_bool(w, 4, true);
    pw_bool(w, 5, true);
    pw_u(w, 7, c->mx.role);
    pw_u(w, 9, MT_HW_HELTEC_V3);
    pw_bool(w, 10, m->rhw_enabled);
    pw_bool(w, 11, x->public_key != NULL);
    pw_u(w, 12, 0x0400 /* ambient lighting: no RGB LED */);
    pw_end(w, mk);
}

static void enc_connection(pw_t *w, const adm_ctx_t *x)
{
    pw_mark_t mk = pw_begin(w, A_GET_CONN_RESP);
    pw_mark_t mw = pw_begin(w, 1);
    pw_mark_t ms = pw_begin(w, 1);
    if (x->ip) pw_fixed32(w, 1, x->ip);
    pw_bool(w, 2, x->wifi_connected);
    pw_bool(w, 3, x->mqtt_connected);
    pw_end(w, ms);
    pw_str(w, 2, x->ssid);
    pw_i32(w, 3, x->wifi_rssi);
    pw_end(w, mw);
    pw_mark_t mb = pw_begin(w, 3);
    pw_bool(w, 3, x->ble_connected);
    pw_end(w, mb);
    pw_end(w, mk);
}

/* ---- Decoders for set_* ------------------------------------------------------------- */

static void set_device(const uint8_t *p, size_t n, edge_config_t *c)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    uint32_t role = 0, rb = 0, ni = c->nodeinfo_interval_s;
    while (pr_next(&r, &f) == 1) {
        if (f.field == 1) role = (uint32_t)f.v;
        if (f.field == 6) rb = (uint32_t)f.v;
        if (f.field == 7) ni = (uint32_t)f.v;
    }
    if (role <= 12 && role != 3) c->mx.role = (uint8_t)role;
    if (rb <= 5) c->rebroadcast = (uint8_t)rb;
    c->nodeinfo_interval_s = ni ? ni : 10800;
}

static void set_position(const uint8_t *p, size_t n, edge_config_t *c)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    bool fixed = false;
    uint32_t every = 0;
    while (pr_next(&r, &f) == 1) {
        if (f.field == 1) every = (uint32_t)f.v;
        if (f.field == 3) fixed = f.v != 0;
    }
    c->mx.pos_interval_s = every ? every : 900;
    if (fixed) c->mx.pos_mode = EDGE_POS_FIXED;
    else if (c->mx.pos_mode == EDGE_POS_FIXED) c->mx.pos_mode = EDGE_POS_OFF;
}

static void set_network(const uint8_t *p, size_t n, edge_config_t *c, adm_result_t *res)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    bool enabled = false;
    while (pr_next(&r, &f) == 1) {
        if (f.field == 1) enabled = f.v != 0;
        if (f.field == 3) pf_str(&f, res->wifi_ssid, sizeof(res->wifi_ssid));
        if (f.field == 4) pf_str(&f, res->wifi_psk, sizeof(res->wifi_psk));
    }
    res->wifi_add = res->wifi_ssid[0] != 0;
    c->link_mode = enabled ? EDGE_LINK_AUTO : EDGE_LINK_AP; /* never Bluetooth-only from afar */
}

static void set_lora(const uint8_t *p, size_t n, edge_config_t *c)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    edge_mesh_ext_t *e = &c->mx;
    bool use_preset = false;
    uint32_t preset = c->preset, region = 0, hop = 0, chn = 0;
    int32_t power = 0;
    float off = 0, ovr = 0;
    bool tx = false, duty = false, boost = false, ign = false, ok = false;
    uint8_t n_ign = 0;
    uint32_t ignore[EDGE_IGNORE_MAX];
    while (pr_next(&r, &f) == 1) {
        switch (f.field) {
        case 1: use_preset = f.v != 0; break;
        case 2: preset = (uint32_t)f.v; break;
        case 6: off = pf_float(&f); break;
        case 7: region = (uint32_t)f.v; break;
        case 8: hop = (uint32_t)f.v; break;
        case 9: tx = f.v != 0; break;
        case 10: power = (int32_t)f.v; break;
        case 11: chn = (uint32_t)f.v; break;
        case 12: duty = f.v != 0; break;
        case 13: boost = f.v != 0; break;
        case 14: ovr = pf_float(&f); break;
        case 103:
            if (f.wt == 0 && n_ign < EDGE_IGNORE_MAX) ignore[n_ign++] = (uint32_t)f.v;
            if (f.wt == 2) {
                pr_t q;
                pr_init(&q, f.data, f.len);
                uint64_t v;
                while (q.pos < q.n && n_ign < EDGE_IGNORE_MAX && pr_varint(&q, &v)) ignore[n_ign++] = (uint32_t)v;
            }
            break;
        case 104: ign = f.v != 0; break;
        case 105: ok = f.v != 0; break;
        default: break;
        }
    }
    if (use_preset && mesh_preset_by_code((uint8_t)preset)) c->preset = (uint8_t)preset;
    if (region == 0 || mesh_region_by_code((uint8_t)region)) c->region = (uint8_t)region;
    c->hop_limit = (uint8_t)(hop >= 1 && hop <= 7 ? hop : 3);
    c->tx_power = (int8_t)(power < 0 ? 0 : power > 30 ? 30 : power);
    c->channel_num = (uint16_t)chn;
    e->freq_offset_hz = (int32_t)off;
    e->override_freq_khz = ovr > 0 ? (uint32_t)(ovr * 1000.0f + 0.5f) : 0;
    e->tx_enabled = tx;
    e->override_duty_cycle = duty;
    e->rx_boosted = boost;
    e->ignore_mqtt = ign;
    e->ok_to_mqtt = ok;
    e->n_ignore = n_ign;
    memcpy(e->ignore, ignore, n_ign * sizeof(uint32_t));
}

static void set_security(const uint8_t *p, size_t n, edge_mesh_mod_t *m)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    uint8_t nk = 0;
    uint8_t keys[EDGE_ADMIN_KEYS_MAX][32];
    bool chan = false;
    while (pr_next(&r, &f) == 1) {
        if (f.field == 3 && f.wt == 2 && f.len == 32 && nk < EDGE_ADMIN_KEYS_MAX) memcpy(keys[nk++], f.data, 32);
        if (f.field == 8) chan = f.v != 0;
    }
    m->n_admin_keys = nk;
    memcpy(m->admin_keys, keys, sizeof(keys));
    m->admin_channel = chan;
    /* Listing a key is the remote way to close remote admin to everyone else. */
    if (nk) m->admin_mode = EDGE_ADMIN_KEYS;
}

static bool set_config(const uint8_t *p, size_t n, edge_config_t *c, edge_mesh_mod_t *m, adm_result_t *res)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    if (pr_next(&r, &f) != 1 || f.wt != 2) return false;
    switch (f.field) {
    case C_DEVICE: set_device(f.data, f.len, c); break;
    case C_POSITION: set_position(f.data, f.len, c); break;
    case C_NETWORK: set_network(f.data, f.len, c, res); break;
    case C_DISPLAY: {
        pr_t q;
        pf_t g;
        pr_init(&q, f.data, f.len);
        uint32_t secs = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) secs = (uint32_t)g.v;
        }
        c->display_timeout_s = (uint16_t)(secs > 3600 ? 3600 : secs);
        break;
    }
    case C_LORA: set_lora(f.data, f.len, c); break;
    case C_SECURITY: set_security(f.data, f.len, m); res->mod_changed = true; return true;
    case C_POWER: case C_BLUETOOTH: case C_SESSIONKEY: case C_DEVICEUI: return true; /* nothing to change */
    default: return false;
    }
    res->cfg_changed = true;
    return true;
}

static int8_t pin_or_none(uint64_t v)
{
    return v == 0 || v > 48 ? -1 : (int8_t)v;
}

static bool set_module(const uint8_t *p, size_t n, edge_config_t *c, edge_mesh_mod_t *m, adm_result_t *res)
{
    pr_t r, q;
    pf_t f, g;
    pr_init(&r, p, n);
    if (pr_next(&r, &f) != 1 || f.wt != 2) return false;
    edge_mesh_ext_t *e = &c->mx;
    pr_init(&q, f.data, f.len);
    switch (f.field) {
    case M_MQTT: {
        edge_mesh_mod_t t = *m;
        t.mqtt_enabled = t.mqtt_encryption = t.mqtt_json = t.mqtt_tls = t.mqtt_proxy_to_client = t.mqtt_map = false;
        t.mqtt_root[0] = 0;
        t.mqtt_map_location = false;
        while (pr_next(&q, &g) == 1) {
            switch (g.field) {
            case 1: t.mqtt_enabled = g.v != 0; break;
            case 2: pf_str(&g, t.mqtt_address, sizeof(t.mqtt_address)); break;
            case 3: pf_str(&g, t.mqtt_username, sizeof(t.mqtt_username)); break;
            case 4: pf_str(&g, t.mqtt_password, sizeof(t.mqtt_password)); break;
            case 5: t.mqtt_encryption = g.v != 0; break;
            case 6: t.mqtt_json = g.v != 0; break;
            case 7: t.mqtt_tls = g.v != 0; break;
            case 8: pf_str(&g, t.mqtt_root, sizeof(t.mqtt_root)); break;
            case 9: t.mqtt_proxy_to_client = g.v != 0; break;
            case 10: t.mqtt_map = g.v != 0; break;
            case 11: {
                pr_t s;
                pf_t h;
                pr_init(&s, g.data, g.len);
                while (pr_next(&s, &h) == 1) {
                    if (h.field == 1) t.mqtt_map_interval_s = (uint32_t)h.v;
                    if (h.field == 2) t.mqtt_map_precision = (uint8_t)h.v;
                    if (h.field == 3) t.mqtt_map_location = h.v != 0;
                }
                break;
            }
            default: break;
            }
        }
        *m = t;
        res->mod_changed = true;
        return true;
    }
    case M_SERIAL:
        e->serial_enabled = false;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) e->serial_enabled = g.v != 0;
            if (g.field == 3) e->serial_rx = pin_or_none(g.v);
            if (g.field == 4) e->serial_tx = pin_or_none(g.v);
            if (g.field == 5) e->serial_baud = mesh_admin_baud_value((uint32_t)g.v);
        }
        break;
    case M_EXTNOTIF: {
        bool en = false, msg = false, bell = false;
        uint32_t out = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) en = g.v != 0;
            if (g.field == 3) out = (uint32_t)g.v;
            if (g.field == 5 || g.field == 10 || g.field == 11) msg = msg || g.v != 0;
            if (g.field == 6 || g.field == 12 || g.field == 13) bell = bell || g.v != 0;
        }
        e->notify_mode = (uint8_t)(!en ? 0 : msg ? 1 : bell ? 2 : 1);
        e->notify_gpio = pin_or_none(out);
        break;
    }
    case M_STOREFWD:
        m->sf_enabled = m->sf_heartbeat = false;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1 || g.field == 6) m->sf_enabled = m->sf_enabled || g.v != 0;
            if (g.field == 2) m->sf_heartbeat = g.v != 0;
            if (g.field == 4) m->sf_return_max = (uint16_t)g.v;
            if (g.field == 5) m->sf_return_window_min = (uint16_t)g.v;
        }
        res->mod_changed = true;
        return true;
    case M_RANGETEST: {
        bool en = false;
        uint32_t every = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) en = g.v != 0;
            if (g.field == 2) every = (uint32_t)g.v;
        }
        e->range_test_interval_s = en ? (every ? every : 60) : 0;
        break;
    }
    case M_TELEMETRY: {
        uint32_t every = 0;
        bool en = true, seen = false;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) every = (uint32_t)g.v;
            if (g.field == 14) {
                en = g.v != 0;
                seen = true;
            }
        }
        (void)seen;
        e->telemetry_interval_s = en ? (every ? every : 1800) : 0;
        break;
    }
    case M_CANNED:
        return true; /* the messages themselves come with set_canned_message_module_messages */
    case M_AUDIO:
        m->audio_enabled = false;
        m->audio_ptt = m->i2s_ws = m->i2s_sd = m->i2s_din = m->i2s_sck = -1;
        m->audio_bitrate = 0;
        while (pr_next(&q, &g) == 1) {
            switch (g.field) {
            case 1: m->audio_enabled = g.v != 0; break;
            case 2: m->audio_ptt = pin_or_none(g.v); break;
            case 3: m->audio_bitrate = (uint8_t)g.v; break;
            case 4: m->i2s_ws = pin_or_none(g.v); break;
            case 5: m->i2s_sd = pin_or_none(g.v); break;
            case 6: m->i2s_din = pin_or_none(g.v); break;
            case 7: m->i2s_sck = pin_or_none(g.v); break;
            default: break;
            }
        }
        res->mod_changed = true;
        return true;
    case M_RHW:
        m->rhw_enabled = m->rhw_undefined_ok = false;
        m->n_pins = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) m->rhw_enabled = g.v != 0;
            if (g.field == 2) m->rhw_undefined_ok = g.v != 0;
            if (g.field == 3 && g.wt == 2 && m->n_pins < EDGE_RHW_PINS_MAX) {
                edge_rhw_pin_t pin = {0};
                pr_t s;
                pf_t h;
                pr_init(&s, g.data, g.len);
                while (pr_next(&s, &h) == 1) {
                    if (h.field == 1) pin.gpio = (uint8_t)h.v;
                    if (h.field == 2) pf_str(&h, pin.name, sizeof(pin.name));
                    if (h.field == 3) pin.type = (uint8_t)h.v;
                }
                m->pins[m->n_pins++] = pin;
            }
        }
        res->mod_changed = true;
        return true;
    case M_NEIGHBOR:
        e->neighbor_info = false;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) e->neighbor_info = g.v != 0;
            if (g.field == 2) e->neighbor_interval_s = (uint32_t)g.v;
        }
        if (e->neighbor_interval_s < 14400) e->neighbor_interval_s = 14400;
        break;
    case M_AMBIENT:
        return true;
    case M_DETECTION:
        e->detect_enabled = false;
        e->detect_name[0] = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) e->detect_enabled = g.v != 0;
            if (g.field == 2) e->detect_min_interval_s = (uint16_t)(g.v > 3600 ? 3600 : g.v);
            if (g.field == 5) pf_str(&g, e->detect_name, sizeof(e->detect_name));
            if (g.field == 6) e->detect_gpio = pin_or_none(g.v);
            if (g.field == 7) e->detect_active_high = g.v == 1 || g.v == 3 || g.v == 5;
        }
        break;
    case M_TAK:
        m->tak_team = m->tak_role = 0;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) m->tak_team = (uint8_t)(g.v < 15 ? g.v : 0);
            if (g.field == 2) m->tak_role = (uint8_t)(g.v < 9 ? g.v : 0);
        }
        res->mod_changed = true;
        return true;
    case M_STATUSMSG: case M_TRAFFIC:
        return true;
    case M_PAX:
        e->pax_enabled = false;
        while (pr_next(&q, &g) == 1) {
            if (g.field == 1) e->pax_enabled = g.v != 0;
            if (g.field == 2) e->pax_interval_s = (uint32_t)g.v;
        }
        if (e->pax_interval_s < 60) e->pax_interval_s = 900;
        break;
    default:
        return false;
    }
    res->cfg_changed = true;
    return true;
}

static bool set_channel(const uint8_t *p, size_t n, edge_config_t *c, edge_mesh_mod_t *m, adm_result_t *res)
{
    pr_t r;
    pf_t f;
    pr_init(&r, p, n);
    uint32_t index = 0, role = 0;
    edge_mesh_channel_t ch = {.position_precision = 32};
    bool up = false, down = false, has_settings = false;
    while (pr_next(&r, &f) == 1) {
        if (f.field == 1) index = (uint32_t)f.v;
        if (f.field == 3) role = (uint32_t)f.v;
        if (f.field == 2 && f.wt == 2) {
            has_settings = true;
            pr_t s;
            pf_t g;
            pr_init(&s, f.data, f.len);
            while (pr_next(&s, &g) == 1) {
                if (g.field == 2 && g.wt == 2 && (g.len == 0 || g.len == 1 || g.len == 16 || g.len == 32)) {
                    memcpy(ch.psk, g.data, g.len);
                    ch.psk_len = (uint8_t)g.len;
                }
                if (g.field == 3) pf_str(&g, ch.name, sizeof(ch.name));
                if (g.field == 5) up = g.v != 0;
                if (g.field == 6) down = g.v != 0;
                if (g.field == 7 && g.wt == 2) {
                    pr_t t;
                    pf_t h;
                    pr_init(&t, g.data, g.len);
                    ch.position_precision = 0;
                    while (pr_next(&t, &h) == 1) {
                        if (h.field == 1) ch.position_precision = (uint8_t)(h.v > 32 ? 32 : h.v);
                    }
                }
            }
        }
    }
    if (index >= EDGE_MESH_MAX_CHANNELS) return false;
    if (role == 0) {
        /* DISABLED: remove it (the primary channel cannot be removed) */
        if (index == 0 || index >= c->n_channels) return true;
        for (uint32_t i = index; i + 1 < c->n_channels; i++) c->channels[i] = c->channels[i + 1];
        c->n_channels--;
        uint8_t lowmask = (uint8_t)((1u << index) - 1);
        m->ch_uplink = (uint8_t)((m->ch_uplink & lowmask) | ((m->ch_uplink >> 1) & ~lowmask));
        m->ch_downlink = (uint8_t)((m->ch_downlink & lowmask) | ((m->ch_downlink >> 1) & ~lowmask));
    } else {
        if (!has_settings || index > c->n_channels) return false;
        c->channels[index] = ch;
        if (index == c->n_channels) c->n_channels++;
        m->ch_uplink = (uint8_t)((m->ch_uplink & ~(1u << index)) | ((up ? 1u : 0u) << index));
        m->ch_downlink = (uint8_t)((m->ch_downlink & ~(1u << index)) | ((down ? 1u : 0u) << index));
    }
    res->cfg_changed = res->mod_changed = true;
    return true;
}

static void set_canned(const uint8_t *p, size_t n, edge_config_t *c)
{
    edge_mesh_ext_t *e = &c->mx;
    e->n_canned = 0;
    size_t start = 0;
    for (size_t i = 0; i <= n && e->n_canned < EDGE_CANNED_MAX; i++) {
        if (i == n || p[i] == '|') {
            size_t len = i - start;
            if (len) {
                if (len >= sizeof(e->canned[0])) len = sizeof(e->canned[0]) - 1;
                memcpy(e->canned[e->n_canned], p + start, len);
                e->canned[e->n_canned][len] = 0;
                e->n_canned++;
            }
            start = i + 1;
        }
    }
}

/* ---- Entry point ---------------------------------------------------------------------- */

static void reply_begin(pw_t *w, adm_result_t *res)
{
    pw_init(w, res->reply, sizeof(res->reply));
}

static void reply_end(pw_t *w, const adm_ctx_t *x, adm_result_t *res)
{
    if (x->passkey) pw_bytes(w, A_PASSKEY, x->passkey, 8);
    res->reply_len = w->overflow ? 0 : w->len;
}

bool mesh_admin_apply(const uint8_t *in, size_t n, const adm_ctx_t *x, edge_config_t *c, edge_mesh_mod_t *m,
                      adm_result_t *res)
{
    memset(res, 0, sizeof(*res));
    res->reboot_s = -1;
    pr_t r;
    pf_t f;
    pr_init(&r, in, n);
    pw_t w;
    int k;
    bool done = false;
    while (!done && (k = pr_next(&r, &f)) == 1) {
        if (f.field == A_PASSKEY) continue;
        done = true;
        switch (f.field) {
        case A_GET_CHANNEL: {
            uint32_t idx = (uint32_t)f.v; /* index + 1 */
            if (!idx || idx > EDGE_MESH_MAX_CHANNELS) return false;
            reply_begin(&w, res);
            pw_mark_t mk = pw_begin(&w, A_GET_CHANNEL_RESP);
            enc_channel(&w, idx - 1, c, m, x->sensitive_ok);
            pw_end(&w, mk);
            reply_end(&w, x, res);
            break;
        }
        case A_GET_OWNER:
            reply_begin(&w, res);
            enc_owner(&w, c, x);
            reply_end(&w, x, res);
            break;
        case A_GET_CONFIG: {
            reply_begin(&w, res);
            pw_mark_t mk = pw_begin(&w, A_GET_CONFIG_RESP);
            if (!enc_config(&w, (uint32_t)f.v, c, m, x)) return false;
            pw_end(&w, mk);
            reply_end(&w, x, res);
            break;
        }
        case A_GET_MODULE: {
            reply_begin(&w, res);
            pw_mark_t mk = pw_begin(&w, A_GET_MODULE_RESP);
            if (!enc_module(&w, (uint32_t)f.v, c, m, x)) return false;
            pw_end(&w, mk);
            reply_end(&w, x, res);
            break;
        }
        case A_GET_CANNED: {
            char all[EDGE_CANNED_MAX * 40 + 8] = "";
            size_t len = 0;
            for (int i = 0; i < c->mx.n_canned; i++) {
                len += (size_t)snprintf(all + len, sizeof(all) - len, "%s%s", i ? "|" : "", c->mx.canned[i]);
                if (len >= sizeof(all)) len = sizeof(all) - 1;
            }
            reply_begin(&w, res);
            pw_bytes(&w, A_GET_CANNED_RESP, all, len);
            reply_end(&w, x, res);
            break;
        }
        case A_GET_METADATA:
            reply_begin(&w, res);
            enc_metadata(&w, c, m, x);
            reply_end(&w, x, res);
            break;
        case A_GET_RINGTONE:
            reply_begin(&w, res);
            pw_bytes(&w, A_GET_RINGTONE_RESP, "", 0);
            reply_end(&w, x, res);
            break;
        case A_GET_CONN:
            reply_begin(&w, res);
            enc_connection(&w, x);
            reply_end(&w, x, res);
            break;
        case A_GET_RHW_PINS: {
            reply_begin(&w, res);
            pw_mark_t mk = pw_begin(&w, A_GET_RHW_PINS_RESP);
            for (int i = 0; i < m->n_pins; i++) {
                pw_mark_t mn = pw_begin(&w, 1);
                pw_u(&w, 1, x->my_node);
                pw_mark_t mp = pw_begin(&w, 2);
                pw_u(&w, 1, m->pins[i].gpio);
                pw_str(&w, 2, m->pins[i].name);
                pw_u(&w, 3, m->pins[i].type);
                pw_end(&w, mp);
                pw_end(&w, mn);
            }
            pw_end(&w, mk);
            reply_end(&w, x, res);
            break;
        }
        case A_SET_HAM: {
            pr_t q;
            pf_t g;
            pr_init(&q, f.data, f.len);
            char call[40] = "", sname[8] = "";
            int32_t power = 0;
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) pf_str(&g, call, sizeof(call));
                if (g.field == 2) power = (int32_t)g.v;
                if (g.field == 4) pf_str(&g, sname, sizeof(sname));
            }
            if (!call[0]) return false;
            c->mx.licensed = true;
            edge_strlcpy(c->long_name, call, sizeof(c->long_name));
            if (sname[0]) edge_strlcpy(c->short_name, sname, sizeof(c->short_name));
            if (power > 0) c->tx_power = (int8_t)(power > 30 ? 30 : power);
            res->cfg_changed = res->owner_changed = true;
            break;
        }
        case A_SET_OWNER: {
            mt_user_t u;
            if (f.wt != 2 || !mt_user_decode(f.data, f.len, &u)) return false;
            if (u.long_name[0]) edge_strlcpy(c->long_name, u.long_name, sizeof(c->long_name));
            if (u.short_name[0]) edge_strlcpy(c->short_name, u.short_name, sizeof(c->short_name));
            c->mx.licensed = u.is_licensed;
            res->cfg_changed = res->owner_changed = true;
            break;
        }
        case A_SET_CHANNEL:
            if (f.wt != 2 || !set_channel(f.data, f.len, c, m, res)) return false;
            break;
        case A_SET_CONFIG:
            if (f.wt != 2 || !set_config(f.data, f.len, c, m, res)) return false;
            break;
        case A_SET_MODULE:
            if (f.wt != 2 || !set_module(f.data, f.len, c, m, res)) return false;
            break;
        case A_SET_CANNED:
            if (f.wt != 2) return false;
            set_canned(f.data, f.len, c);
            res->cfg_changed = true;
            break;
        case A_SET_RINGTONE:
            break; /* no buzzer */
        case A_REMOVE_NODE: res->remove_node = (uint32_t)f.v; break;
        case A_SET_FAV: res->fav_set = (uint32_t)f.v; break;
        case A_REMOVE_FAV: res->fav_clear = (uint32_t)f.v; break;
        case A_SET_IGNORED: res->ign_set = (uint32_t)f.v; break;
        case A_REMOVE_IGNORED: res->ign_clear = (uint32_t)f.v; break;
        case A_TOGGLE_MUTED: break;
        case A_SET_FIXED_POS: {
            mt_position_t pos;
            if (f.wt != 2 || !mt_position_decode(f.data, f.len, &pos) || !pos.has_lat || !pos.has_lon) return false;
            c->mx.pos_mode = EDGE_POS_FIXED;
            c->mx.pos_lat_i = pos.lat_i;
            c->mx.pos_lon_i = pos.lon_i;
            c->mx.pos_alt = pos.has_alt ? pos.alt : 0;
            res->cfg_changed = true;
            break;
        }
        case A_REMOVE_FIXED_POS:
            c->mx.pos_mode = EDGE_POS_OFF;
            res->cfg_changed = true;
            break;
        case A_SET_TIME:
            res->set_time = true;
            res->time = (uint32_t)f.v;
            break;
        case A_BEGIN_EDIT: case A_COMMIT_EDIT:
            break; /* every change applies on its own */
        case A_ADD_CONTACT: {
            pr_t q;
            pf_t g;
            pr_init(&q, f.data, f.len);
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) res->contact_num = (uint32_t)g.v;
                if (g.field == 2 && g.wt == 2) {
                    mt_user_t u;
                    if (mt_user_decode(g.data, g.len, &u)) {
                        edge_strlcpy(res->contact_long, u.long_name, sizeof(res->contact_long));
                        edge_strlcpy(res->contact_short, u.short_name, sizeof(res->contact_short));
                        res->contact_hw = u.hw_model;
                        res->contact_has_key = u.has_public_key;
                        if (u.has_public_key) memcpy(res->contact_key, u.public_key, 32);
                    }
                }
            }
            res->add_contact = res->contact_num != 0;
            break;
        }
        case A_FACTORY_DEVICE: res->factory_device = true; break;
        case A_FACTORY_CONFIG: res->factory_config = true; break;
        case A_NODEDB_RESET: res->nodedb_reset = true; break;
        case A_REBOOT: res->reboot_s = (int32_t)f.v < 0 ? 0 : (int32_t)f.v; break;
        case A_SHUTDOWN: res->unsupported = true; break; /* no power switch on the Heltec V3 */
        default:
            res->unsupported = true;
            break;
        }
    }
    return true;
}

/* ---- Request builders ------------------------------------------------------------------ */

size_t mesh_admin_build_get_config(uint32_t type, const uint8_t *passkey, uint8_t *out, size_t cap)
{
    pw_t w;
    pw_init(&w, out, cap);
    pw_u_always(&w, A_GET_CONFIG, type);
    if (passkey) pw_bytes(&w, A_PASSKEY, passkey, 8);
    return w.overflow ? 0 : w.len;
}

size_t mesh_admin_build_get_module(uint32_t type, const uint8_t *passkey, uint8_t *out, size_t cap)
{
    pw_t w;
    pw_init(&w, out, cap);
    pw_u_always(&w, A_GET_MODULE, type);
    if (passkey) pw_bytes(&w, A_PASSKEY, passkey, 8);
    return w.overflow ? 0 : w.len;
}

size_t mesh_admin_build_get_channel(uint32_t index, uint8_t *out, size_t cap)
{
    pw_t w;
    pw_init(&w, out, cap);
    pw_u_always(&w, A_GET_CHANNEL, index + 1);
    return w.overflow ? 0 : w.len;
}
