#include "mesh_pb_ext.h"

#include "pb_wire.h"

/* ---- MeshPacket / ServiceEnvelope ----------------------------------------------------- */

static void enc_packet(pw_t *w, const mt_packet_t *p)
{
    pw_fixed32(w, 1, p->from);
    pw_fixed32(w, 2, p->to);
    pw_u(w, 3, p->channel);
    if (p->payload) pw_bytes(w, p->decoded ? 4 : 5, p->payload, p->payload_len);
    pw_fixed32(w, 6, p->id);
    if (p->rx_time) pw_fixed32(w, 7, p->rx_time);
    pw_float(w, 8, p->rx_snr);
    pw_u(w, 9, p->hop_limit);
    pw_bool(w, 10, p->want_ack);
    pw_u(w, 11, p->priority);
    pw_i32(w, 12, p->rx_rssi);
    pw_bool(w, 14, p->via_mqtt);
    pw_u(w, 15, p->hop_start);
    pw_bool(w, 17, p->pki_encrypted);
    pw_u(w, 18, p->next_hop);
    pw_u(w, 19, p->relay_node);
}

size_t mt_envelope_encode(const mt_envelope_t *e, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    if (e->has_packet) {
        pw_mark_t m = pw_begin(&w, 1);
        enc_packet(&w, &e->packet);
        pw_end(&w, m);
    }
    pw_str(&w, 2, e->channel_id);
    pw_str(&w, 3, e->gateway_id);
    return w.overflow ? 0 : w.len;
}

static bool dec_packet(const uint8_t *buf, size_t n, mt_packet_t *p)
{
    memset(p, 0, sizeof(*p));
    pr_t r;
    pf_t f;
    pr_init(&r, buf, n);
    int k;
    while ((k = pr_next(&r, &f)) == 1) {
        switch (f.field) {
        case 1: p->from = (uint32_t)f.v; break;
        case 2: p->to = (uint32_t)f.v; break;
        case 3: p->channel = (uint32_t)f.v; break;
        case 4: if (f.wt == 2) { p->decoded = true; p->payload = f.data; p->payload_len = f.len; } break;
        case 5: if (f.wt == 2) { p->decoded = false; p->payload = f.data; p->payload_len = f.len; } break;
        case 6: p->id = (uint32_t)f.v; break;
        case 7: p->rx_time = (uint32_t)f.v; break;
        case 8: p->rx_snr = pf_float(&f); break;
        case 9: p->hop_limit = (uint32_t)f.v; break;
        case 10: p->want_ack = f.v != 0; break;
        case 11: p->priority = (uint32_t)f.v; break;
        case 12: p->rx_rssi = (int32_t)f.v; break;
        case 14: p->via_mqtt = f.v != 0; break;
        case 15: p->hop_start = (uint32_t)f.v; break;
        case 17: p->pki_encrypted = f.v != 0; break;
        case 18: p->next_hop = (uint32_t)f.v; break;
        case 19: p->relay_node = (uint32_t)f.v; break;
        default: break;
        }
    }
    return k == 0;
}

bool mt_envelope_decode(const uint8_t *buf, size_t n, mt_envelope_t *out)
{
    memset(out, 0, sizeof(*out));
    pr_t r;
    pf_t f;
    pr_init(&r, buf, n);
    int k;
    while ((k = pr_next(&r, &f)) == 1) {
        if (f.field == 1 && f.wt == 2) out->has_packet = dec_packet(f.data, f.len, &out->packet);
        if (f.field == 2) pf_str(&f, out->channel_id, sizeof(out->channel_id));
        if (f.field == 3) pf_str(&f, out->gateway_id, sizeof(out->gateway_id));
    }
    return k == 0 && out->has_packet;
}

/* ---- MapReport --------------------------------------------------------------------------- */

size_t mt_mapreport_encode(const mt_mapreport_t *m, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_str(&w, 1, m->long_name);
    pw_str(&w, 2, m->short_name);
    pw_u(&w, 3, m->role);
    pw_u(&w, 4, m->hw_model);
    pw_str(&w, 5, m->firmware_version);
    pw_u(&w, 6, m->region);
    pw_u(&w, 7, m->modem_preset);
    pw_bool(&w, 8, m->has_default_channel);
    if (m->has_position) {
        pw_fixed32(&w, 9, (uint32_t)m->lat_i);
        pw_fixed32(&w, 10, (uint32_t)m->lon_i);
        pw_i32(&w, 11, m->altitude);
        pw_u(&w, 12, m->position_precision);
    }
    pw_u(&w, 13, m->online_nodes);
    pw_bool(&w, 14, m->opted_location);
    return w.overflow ? 0 : w.len;
}

/* ---- StoreAndForward ------------------------------------------------------------------- */

size_t mt_sf_encode_stats(const mt_sf_stats_t *s, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, MT_SF_ROUTER_STATS);
    pw_mark_t m = pw_begin(&w, 2);
    pw_u(&w, 1, s->messages_total);
    pw_u(&w, 2, s->messages_saved);
    pw_u(&w, 3, s->messages_max);
    pw_u(&w, 4, s->up_time);
    pw_u(&w, 5, s->requests);
    pw_u(&w, 6, s->requests_history);
    pw_bool(&w, 7, s->heartbeat);
    pw_u(&w, 8, s->return_max);
    pw_u(&w, 9, s->return_window);
    pw_end(&w, m);
    return w.overflow ? 0 : w.len;
}

size_t mt_sf_encode_history(uint32_t rr, uint32_t messages, uint32_t window_ms, uint32_t last_request, uint8_t *buf,
                            size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, rr);
    pw_mark_t m = pw_begin(&w, 3);
    pw_u(&w, 1, messages);
    pw_u(&w, 2, window_ms);
    pw_u(&w, 3, last_request);
    pw_end(&w, m);
    return w.overflow ? 0 : w.len;
}

size_t mt_sf_encode_heartbeat(uint32_t period, uint32_t secondary, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, 2 /* ROUTER_HEARTBEAT */);
    pw_mark_t m = pw_begin(&w, 4);
    pw_u(&w, 1, period);
    pw_u(&w, 2, secondary);
    pw_end(&w, m);
    return w.overflow ? 0 : w.len;
}

size_t mt_sf_encode_text(uint32_t rr, const uint8_t *text, size_t len, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, rr);
    pw_bytes(&w, 5, text, len);
    return w.overflow ? 0 : w.len;
}

size_t mt_sf_encode_rr(uint32_t rr, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u_always(&w, 1, rr);
    return w.overflow ? 0 : w.len;
}

/* ---- HardwareMessage ------------------------------------------------------------------- */

size_t mt_hw_encode(const mt_hw_t *h, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, h->type);
    pw_u(&w, 2, h->mask);
    pw_u(&w, 3, h->value);
    return w.overflow ? 0 : w.len;
}

bool mt_hw_decode(const uint8_t *buf, size_t n, mt_hw_t *out)
{
    memset(out, 0, sizeof(*out));
    pr_t r;
    pf_t f;
    pr_init(&r, buf, n);
    int k;
    while ((k = pr_next(&r, &f)) == 1) {
        if (f.field == 1) out->type = (uint32_t)f.v;
        if (f.field == 2) out->mask = f.v;
        if (f.field == 3) out->value = f.v;
    }
    return k == 0;
}

/* ---- ATAK -------------------------------------------------------------------------------- */

static uint32_t zz32(int32_t v)
{
    return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
}

static int32_t unzz32(uint64_t v)
{
    uint32_t u = (uint32_t)v;
    return (int32_t)(u >> 1) ^ -(int32_t)(u & 1);
}

size_t mt_takv2_encode(const mt_tak_t *t, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_u(&w, 1, t->cot_type);
    pw_u(&w, 2, t->how);
    pw_str(&w, 3, t->callsign);
    pw_u(&w, 4, t->team);
    pw_u(&w, 5, t->role);
    if (t->has_pos) {
        pw_fixed32(&w, 6, (uint32_t)t->lat_i);
        pw_fixed32(&w, 7, (uint32_t)t->lon_i);
        pw_u(&w, 8, zz32(t->alt));
        pw_u(&w, 9, t->speed);
        pw_u(&w, 10, t->course);
    }
    pw_u(&w, 11, t->battery);
    pw_u(&w, 12, t->geo_src);
    pw_u(&w, 13, t->alt_src);
    pw_str(&w, 14, t->uid);
    pw_str(&w, 15, t->device_callsign);
    if (t->has_chat) {
        pw_mark_t m = pw_begin(&w, 31);
        pw_str(&w, 1, t->chat);
        pw_str(&w, 2, t->chat_to);
        pw_str(&w, 3, t->chat_to_callsign);
        pw_end(&w, m);
    }
    return w.overflow ? 0 : w.len;
}

static void dec_chat(const pf_t *f, mt_tak_t *out)
{
    pr_t q;
    pf_t g;
    pr_init(&q, f->data, f->len);
    out->has_chat = true;
    while (pr_next(&q, &g) == 1) {
        if (g.field == 1) pf_str(&g, out->chat, sizeof(out->chat));
        if (g.field == 2) pf_str(&g, out->chat_to, sizeof(out->chat_to));
        if (g.field == 3) pf_str(&g, out->chat_to_callsign, sizeof(out->chat_to_callsign));
    }
}

bool mt_takv2_decode(const uint8_t *buf, size_t n, mt_tak_t *out)
{
    memset(out, 0, sizeof(*out));
    out->v2 = true;
    pr_t r;
    pf_t f;
    pr_init(&r, buf, n);
    int k;
    bool lat = false, lon = false;
    while ((k = pr_next(&r, &f)) == 1) {
        switch (f.field) {
        case 1: out->cot_type = (uint32_t)f.v; break;
        case 2: out->how = (uint32_t)f.v; break;
        case 3: pf_str(&f, out->callsign, sizeof(out->callsign)); break;
        case 4: out->team = (uint32_t)f.v; break;
        case 5: out->role = (uint32_t)f.v; break;
        case 6: out->lat_i = (int32_t)(uint32_t)f.v; lat = true; break;
        case 7: out->lon_i = (int32_t)(uint32_t)f.v; lon = true; break;
        case 8: out->alt = unzz32(f.v); break;
        case 9: out->speed = (uint32_t)f.v; break;
        case 10: out->course = (uint32_t)f.v; break;
        case 11: out->battery = (uint32_t)f.v; break;
        case 12: out->geo_src = (uint32_t)f.v; break;
        case 13: out->alt_src = (uint32_t)f.v; break;
        case 14: pf_str(&f, out->uid, sizeof(out->uid)); break;
        case 15: pf_str(&f, out->device_callsign, sizeof(out->device_callsign)); break;
        case 31: if (f.wt == 2) dec_chat(&f, out); break;
        default: break;
        }
    }
    out->has_pos = lat && lon && (out->lat_i || out->lon_i);
    return k == 0;
}

bool mt_tak_port78_decode(const uint8_t *buf, size_t n, mt_tak_t *out)
{
    memset(out, 0, sizeof(*out));
    out->v2 = true;
    if (n < 1) return false;
    if (buf[0] != MT_TAK_RAW_FLAGS) {
        out->compressed = true;
        return true;
    }
    return mt_takv2_decode(buf + 1, n - 1, out);
}

bool mt_takv1_decode(const uint8_t *buf, size_t n, mt_tak_t *out)
{
    memset(out, 0, sizeof(*out));
    pr_t r;
    pf_t f;
    pr_init(&r, buf, n);
    int k;
    while ((k = pr_next(&r, &f)) == 1) {
        if (f.field == 1) out->compressed = f.v != 0;
        if (f.wt != 2) continue;
        pr_t q;
        pf_t g;
        pr_init(&q, f.data, f.len);
        switch (f.field) {
        case 2:
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) pf_str(&g, out->callsign, sizeof(out->callsign));
                if (g.field == 2) pf_str(&g, out->device_callsign, sizeof(out->device_callsign));
            }
            break;
        case 3:
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) out->role = (uint32_t)g.v;
                if (g.field == 2) out->team = (uint32_t)g.v;
            }
            break;
        case 4:
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) out->battery = (uint32_t)g.v;
            }
            break;
        case 5: {
            bool lat = false, lon = false;
            while (pr_next(&q, &g) == 1) {
                if (g.field == 1) { out->lat_i = (int32_t)(uint32_t)g.v; lat = true; }
                if (g.field == 2) { out->lon_i = (int32_t)(uint32_t)g.v; lon = true; }
                if (g.field == 3) out->alt = (int32_t)g.v;
                if (g.field == 4) out->speed = (uint32_t)g.v;
                if (g.field == 5) out->course = (uint32_t)g.v;
            }
            out->has_pos = lat && lon;
            break;
        }
        case 6:
            dec_chat(&f, out);
            break;
        default:
            break;
        }
    }
    if (out->compressed) {
        /* strings are unishox2-compressed by the plugin: not shown as text */
        out->callsign[0] = out->device_callsign[0] = 0;
        if (out->has_chat) out->chat[0] = 0;
    }
    return k == 0;
}

size_t mt_takv1_encode(const mt_tak_t *t, uint8_t *buf, size_t cap)
{
    pw_t w;
    pw_init(&w, buf, cap);
    pw_mark_t m = pw_begin(&w, 2);
    pw_str(&w, 1, t->callsign);
    pw_str(&w, 2, t->device_callsign);
    pw_end(&w, m);
    m = pw_begin(&w, 3);
    pw_u(&w, 1, t->role);
    pw_u(&w, 2, t->team);
    pw_end(&w, m);
    m = pw_begin(&w, 4);
    pw_u(&w, 1, t->battery);
    pw_end(&w, m);
    if (t->has_chat) {
        m = pw_begin(&w, 6);
        pw_str(&w, 1, t->chat);
        pw_str(&w, 2, t->chat_to);
        pw_end(&w, m);
    } else if (t->has_pos) {
        m = pw_begin(&w, 5);
        pw_fixed32(&w, 1, (uint32_t)t->lat_i);
        pw_fixed32(&w, 2, (uint32_t)t->lon_i);
        pw_i32(&w, 3, t->alt);
        pw_u(&w, 4, t->speed);
        pw_u(&w, 5, t->course);
        pw_end(&w, m);
    }
    return w.overflow ? 0 : w.len;
}
