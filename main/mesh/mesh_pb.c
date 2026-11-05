#include "mesh_pb.h"

#include <string.h>

/* ---- Wire helpers ---------------------------------------------------------- */

enum { WT_VARINT = 0, WT_I64 = 1, WT_LEN = 2, WT_I32 = 5 };

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
} pbw_t;

static void put(pbw_t *w, uint8_t b)
{
    if (w->len >= w->cap) {
        w->overflow = true;
        return;
    }
    w->buf[w->len++] = b;
}

static void put_varint(pbw_t *w, uint64_t v)
{
    do {
        uint8_t b = v & 0x7f;
        v >>= 7;
        put(w, v ? (b | 0x80) : b);
    } while (v);
}

static void put_tag(pbw_t *w, uint32_t field, uint8_t wt)
{
    put_varint(w, ((uint64_t)field << 3) | wt);
}

static void w_varint(pbw_t *w, uint32_t field, uint64_t v)
{
    put_tag(w, field, WT_VARINT);
    put_varint(w, v);
}

static void w_fixed32(pbw_t *w, uint32_t field, uint32_t v)
{
    put_tag(w, field, WT_I32);
    for (int i = 0; i < 4; i++) put(w, (uint8_t)(v >> (8 * i)));
}

static void w_bytes(pbw_t *w, uint32_t field, const void *data, size_t n)
{
    put_tag(w, field, WT_LEN);
    put_varint(w, n);
    const uint8_t *p = data;
    for (size_t i = 0; i < n; i++) put(w, p[i]);
}

static void w_string(pbw_t *w, uint32_t field, const char *s)
{
    if (s && s[0]) w_bytes(w, field, s, strlen(s));
}

static void w_float(pbw_t *w, uint32_t field, float f)
{
    uint32_t v;
    memcpy(&v, &f, 4);
    w_fixed32(w, field, v);
}

/* Length-delimited sub-message: encode into a scratch buffer, then copy. */
static void w_sub(pbw_t *w, uint32_t field, const uint8_t *data, size_t n, bool ok)
{
    if (!ok) {
        w->overflow = true;
        return;
    }
    w_bytes(w, field, data, n);
}

static size_t varint_len(uint64_t v)
{
    size_t n = 1;
    while (v >= 0x80) {
        v >>= 7;
        n++;
    }
    return n;
}

static void w_packed_fixed32(pbw_t *w, uint32_t field, const uint32_t *v, size_t n)
{
    if (!n) return;
    put_tag(w, field, 2);
    put_varint(w, 4 * n);
    for (size_t i = 0; i < n; i++) {
        for (int k = 0; k < 4; k++) put(w, (uint8_t)(v[i] >> (8 * k)));
    }
}

static void w_packed_int32(pbw_t *w, uint32_t field, const int8_t *v, size_t n)
{
    if (!n) return;
    size_t len = 0;
    for (size_t i = 0; i < n; i++) len += varint_len((uint64_t)(int64_t)v[i]);
    put_tag(w, field, 2);
    put_varint(w, len);
    for (size_t i = 0; i < n; i++) put_varint(w, (uint64_t)(int64_t)v[i]);
}

typedef struct {
    const uint8_t *p;
    size_t n;
    size_t pos;
} pbr_t;

typedef struct {
    uint32_t field;
    uint8_t wt;
    uint64_t v;          /* varint / fixed value */
    const uint8_t *data; /* WT_LEN */
    size_t len;
} pbf_t;

static bool get_varint(pbr_t *r, uint64_t *v)
{
    uint64_t x = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        if (r->pos >= r->n) return false;
        uint8_t b = r->p[r->pos++];
        x |= (uint64_t)(b & 0x7f) << shift;
        if (!(b & 0x80)) {
            *v = x;
            return true;
        }
    }
    return false;
}

/* Returns 1 for a field, 0 at end of buffer, -1 on malformed input. */
static int next_field(pbr_t *r, pbf_t *f)
{
    if (r->pos >= r->n) return 0;
    uint64_t tag;
    if (!get_varint(r, &tag)) return -1;
    f->field = (uint32_t)(tag >> 3);
    f->wt = tag & 7;
    if (f->field == 0) return -1;
    switch (f->wt) {
    case WT_VARINT:
        return get_varint(r, &f->v) ? 1 : -1;
    case WT_I64:
        if (r->pos + 8 > r->n) return -1;
        f->v = 0;
        for (int i = 0; i < 8; i++) f->v |= (uint64_t)r->p[r->pos + i] << (8 * i);
        r->pos += 8;
        return 1;
    case WT_I32:
        if (r->pos + 4 > r->n) return -1;
        f->v = 0;
        for (int i = 0; i < 4; i++) f->v |= (uint64_t)r->p[r->pos + i] << (8 * i);
        r->pos += 4;
        return 1;
    case WT_LEN: {
        uint64_t len;
        if (!get_varint(r, &len) || len > r->n - r->pos) return -1;
        f->data = &r->p[r->pos];
        f->len = (size_t)len;
        r->pos += (size_t)len;
        return 1;
    }
    default:
        return -1; /* groups are not used by Meshtastic */
    }
}

static float f_float(const pbf_t *f)
{
    uint32_t v = (uint32_t)f->v;
    float x;
    memcpy(&x, &v, 4);
    return x;
}

/* Appends one or many (packed) fixed32 values. */
static void r_fixed32s(const pbf_t *f, uint32_t *arr, uint8_t *n, uint8_t cap)
{
    if (f->wt == WT_I32) {
        if (*n < cap) arr[(*n)++] = (uint32_t)f->v;
    } else if (f->wt == WT_LEN) {
        for (size_t i = 0; i + 4 <= f->len && *n < cap; i += 4) {
            arr[(*n)++] = (uint32_t)f->data[i] | (uint32_t)f->data[i + 1] << 8 | (uint32_t)f->data[i + 2] << 16 |
                          (uint32_t)f->data[i + 3] << 24;
        }
    }
}

/* Appends one or many (packed) int32 varints, clamped to int8 (SNR x 4). */
static void r_int8s(const pbf_t *f, int8_t *arr, uint8_t *n, uint8_t cap)
{
    if (f->wt == WT_VARINT) {
        if (*n < cap) arr[(*n)++] = (int8_t)(int32_t)f->v;
    } else if (f->wt == WT_LEN) {
        pbr_t r = {f->data, f->len, 0};
        uint64_t v;
        while (r.pos < r.n && *n < cap && get_varint(&r, &v)) arr[(*n)++] = (int8_t)(int32_t)v;
    }
}

static void copy_str(char *dst, size_t cap, const pbf_t *f)
{
    size_t n = f->len < cap - 1 ? f->len : cap - 1;
    memcpy(dst, f->data, n);
    dst[n] = '\0';
}

/* ---- Header ---------------------------------------------------------------- */

static void le32(uint8_t *o, uint32_t v)
{
    o[0] = (uint8_t)v;
    o[1] = (uint8_t)(v >> 8);
    o[2] = (uint8_t)(v >> 16);
    o[3] = (uint8_t)(v >> 24);
}

static uint32_t rd32(const uint8_t *i)
{
    return (uint32_t)i[0] | ((uint32_t)i[1] << 8) | ((uint32_t)i[2] << 16) | ((uint32_t)i[3] << 24);
}

void mt_header_write(const mt_header_t *h, uint8_t out[16])
{
    le32(out, h->to);
    le32(out + 4, h->from);
    le32(out + 8, h->id);
    out[12] = h->flags;
    out[13] = h->channel;
    out[14] = h->next_hop;
    out[15] = h->relay_node;
}

void mt_header_read(const uint8_t in[16], mt_header_t *h)
{
    h->to = rd32(in);
    h->from = rd32(in + 4);
    h->id = rd32(in + 8);
    h->flags = in[12];
    h->channel = in[13];
    h->next_hop = in[14];
    h->relay_node = in[15];
}

uint8_t mt_make_flags(uint8_t hop_limit, uint8_t hop_start, bool want_ack)
{
    return (uint8_t)((hop_limit & MT_FLAG_HOP_LIMIT_MASK) | (want_ack ? MT_FLAG_WANT_ACK : 0) |
                     ((hop_start << MT_FLAG_HOP_START_SHIFT) & MT_FLAG_HOP_START_MASK));
}

/* ---- Data ------------------------------------------------------------------ */

bool mt_data_decode(const uint8_t *buf, size_t n, mt_data_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        switch (f.field) {
        case 1: if (f.wt == WT_VARINT) out->portnum = (uint32_t)f.v; break;
        case 2: if (f.wt == WT_LEN) { out->payload = f.data; out->payload_len = f.len; } break;
        case 3: if (f.wt == WT_VARINT) out->want_response = f.v != 0; break;
        case 4: if (f.wt == WT_I32) out->dest = (uint32_t)f.v; break;
        case 5: if (f.wt == WT_I32) out->source = (uint32_t)f.v; break;
        case 6: if (f.wt == WT_I32) out->request_id = (uint32_t)f.v; break;
        case 7: if (f.wt == WT_I32) out->reply_id = (uint32_t)f.v; break;
        case 8: if (f.wt == WT_I32) out->emoji = (uint32_t)f.v; break;
        case 9: if (f.wt == WT_VARINT) { out->has_bitfield = true; out->bitfield = (uint32_t)f.v; } break;
        default: break;
        }
    }
    /* A wrong key yields noise: reject anything malformed or without a port. */
    return rc == 0 && out->portnum != MT_PORT_UNKNOWN;
}

size_t mt_data_encode(const mt_data_t *d, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_varint(&w, 1, d->portnum);
    if (d->payload_len) w_bytes(&w, 2, d->payload, d->payload_len);
    if (d->want_response) w_varint(&w, 3, 1);
    if (d->dest) w_fixed32(&w, 4, d->dest);
    if (d->source) w_fixed32(&w, 5, d->source);
    if (d->request_id) w_fixed32(&w, 6, d->request_id);
    if (d->reply_id) w_fixed32(&w, 7, d->reply_id);
    if (d->emoji) w_fixed32(&w, 8, d->emoji);
    if (d->has_bitfield) w_varint(&w, 9, d->bitfield);
    return w.overflow ? 0 : w.len;
}

/* ---- User ------------------------------------------------------------------ */

bool mt_user_decode(const uint8_t *buf, size_t n, mt_user_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        switch (f.field) {
        case 1: if (f.wt == WT_LEN) copy_str(out->id, sizeof(out->id), &f); break;
        case 2: if (f.wt == WT_LEN) copy_str(out->long_name, sizeof(out->long_name), &f); break;
        case 3: if (f.wt == WT_LEN) copy_str(out->short_name, sizeof(out->short_name), &f); break;
        case 5: if (f.wt == WT_VARINT) out->hw_model = (uint32_t)f.v; break;
        case 6: if (f.wt == WT_VARINT) out->is_licensed = f.v != 0; break;
        case 7: if (f.wt == WT_VARINT) out->role = (uint32_t)f.v; break;
        case 8:
            if (f.wt == WT_LEN && f.len == 32) {
                out->has_public_key = true;
                memcpy(out->public_key, f.data, 32);
            }
            break;
        default: break;
        }
    }
    return rc == 0;
}

size_t mt_user_encode(const mt_user_t *u, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_string(&w, 1, u->id);
    w_string(&w, 2, u->long_name);
    w_string(&w, 3, u->short_name);
    if (u->hw_model) w_varint(&w, 5, u->hw_model);
    if (u->is_licensed) w_varint(&w, 6, 1);
    if (u->role) w_varint(&w, 7, u->role);
    if (u->has_public_key) w_bytes(&w, 8, u->public_key, 32);
    return w.overflow ? 0 : w.len;
}

/* ---- Position -------------------------------------------------------------- */

bool mt_position_decode(const uint8_t *buf, size_t n, mt_position_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        switch (f.field) {
        case 1: if (f.wt == WT_I32) { out->has_lat = true; out->lat_i = (int32_t)(uint32_t)f.v; } break;
        case 2: if (f.wt == WT_I32) { out->has_lon = true; out->lon_i = (int32_t)(uint32_t)f.v; } break;
        case 3: if (f.wt == WT_VARINT) { out->has_alt = true; out->alt = (int32_t)(uint32_t)f.v; } break;
        case 4: if (f.wt == WT_I32) out->time = (uint32_t)f.v; break;
        case 5: if (f.wt == WT_VARINT) out->location_source = (uint32_t)f.v; break;
        case 23: if (f.wt == WT_VARINT) out->precision_bits = (uint32_t)f.v; break;
        default: break;
        }
    }
    return rc == 0;
}

int32_t mt_position_truncate(int32_t coord_i, uint32_t bits)
{
    if (bits == 0 || bits >= 32) return coord_i;
    uint32_t mask = 0xffffffffu << (32 - bits);
    uint32_t v = ((uint32_t)coord_i & mask) + (1u << (31 - bits));
    return (int32_t)v;
}

size_t mt_position_encode(const mt_position_t *p, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    if (p->has_lat) w_fixed32(&w, 1, (uint32_t)p->lat_i);
    if (p->has_lon) w_fixed32(&w, 2, (uint32_t)p->lon_i);
    if (p->has_alt) w_varint(&w, 3, (uint64_t)(int64_t)p->alt);
    if (p->time) w_fixed32(&w, 4, p->time);
    if (p->location_source) w_varint(&w, 5, p->location_source);
    if (p->precision_bits) w_varint(&w, 23, p->precision_bits);
    return w.overflow ? 0 : w.len;
}

/* ---- Telemetry ------------------------------------------------------------ */

bool mt_telemetry_decode(const uint8_t *buf, size_t n, mt_telemetry_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.field == 1 && f.wt == WT_I32) out->time = (uint32_t)f.v;
        if (f.field == 2 && f.wt == WT_LEN) {
            out->has_device = true;
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                switch (g.field) {
                case 1: if (g.wt == WT_VARINT) { out->has_battery = true; out->battery_level = (uint32_t)g.v; } break;
                case 2: if (g.wt == WT_I32) { out->has_voltage = true; out->voltage = f_float(&g); } break;
                case 3: if (g.wt == WT_I32) { out->has_ch_util = true; out->channel_utilization = f_float(&g); } break;
                case 4: if (g.wt == WT_I32) { out->has_air_util = true; out->air_util_tx = f_float(&g); } break;
                case 5: if (g.wt == WT_VARINT) { out->has_uptime = true; out->uptime_seconds = (uint32_t)g.v; } break;
                default: break;
                }
            }
        }
        if (f.field == 3 && f.wt == WT_LEN) {
            out->has_env = true;
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.wt != WT_I32) continue;
                if (g.field == 1) { out->has_temp = true; out->temperature = f_float(&g); }
                if (g.field == 2) { out->has_hum = true; out->relative_humidity = f_float(&g); }
                if (g.field == 3) { out->has_press = true; out->barometric_pressure = f_float(&g); }
            }
        }
    }
    return rc == 0;
}

size_t mt_telemetry_encode(const mt_telemetry_t *t, uint8_t *buf, size_t cap)
{
    uint8_t sub[48];
    pbw_t d = {sub, sizeof(sub), 0, false};
    if (t->has_battery) w_varint(&d, 1, t->battery_level);
    if (t->has_voltage) w_float(&d, 2, t->voltage);
    if (t->has_ch_util) w_float(&d, 3, t->channel_utilization);
    if (t->has_air_util) w_float(&d, 4, t->air_util_tx);
    if (t->has_uptime) w_varint(&d, 5, t->uptime_seconds);
    pbw_t w = {buf, cap, 0, false};
    if (t->time) w_fixed32(&w, 1, t->time);
    w_sub(&w, 2, sub, d.len, !d.overflow);
    return w.overflow ? 0 : w.len;
}

/* ---- RouteDiscovery ------------------------------------------------------- */

bool mt_route_decode(const uint8_t *buf, size_t n, mt_route_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        switch (f.field) {
        case 1: r_fixed32s(&f, out->route, &out->n_route, MT_MAX_ROUTE); break;
        case 2: r_int8s(&f, out->snr_towards, &out->n_snr_towards, MT_MAX_ROUTE + 1); break;
        case 3: r_fixed32s(&f, out->route_back, &out->n_route_back, MT_MAX_ROUTE); break;
        case 4: r_int8s(&f, out->snr_back, &out->n_snr_back, MT_MAX_ROUTE + 1); break;
        default: break;
        }
    }
    return rc == 0;
}

size_t mt_route_encode(const mt_route_t *rt, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_packed_fixed32(&w, 1, rt->route, rt->n_route);
    w_packed_int32(&w, 2, rt->snr_towards, rt->n_snr_towards);
    w_packed_fixed32(&w, 3, rt->route_back, rt->n_route_back);
    w_packed_int32(&w, 4, rt->snr_back, rt->n_snr_back);
    return w.overflow ? 0 : w.len;
}

/* ---- NeighborInfo --------------------------------------------------------- */

bool mt_neighborinfo_decode(const uint8_t *buf, size_t n, mt_neighborinfo_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.field == 1 && f.wt == WT_VARINT) out->node_id = (uint32_t)f.v;
        if (f.field == 2 && f.wt == WT_VARINT) out->last_sent_by_id = (uint32_t)f.v;
        if (f.field == 3 && f.wt == WT_VARINT) out->interval_secs = (uint32_t)f.v;
        if (f.field == 4 && f.wt == WT_LEN && out->n < MT_MAX_NEIGHBORS) {
            mt_neighbor_t *nb = &out->nb[out->n++];
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.field == 1 && g.wt == WT_VARINT) nb->node_id = (uint32_t)g.v;
                if (g.field == 2 && g.wt == WT_I32) nb->snr = f_float(&g);
                if (g.field == 3 && g.wt == WT_I32) nb->last_rx_time = (uint32_t)g.v;
                if (g.field == 4 && g.wt == WT_VARINT) nb->interval_secs = (uint32_t)g.v;
            }
        }
    }
    return rc == 0;
}

size_t mt_neighborinfo_encode(const mt_neighborinfo_t *ni, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_varint(&w, 1, ni->node_id);
    if (ni->last_sent_by_id) w_varint(&w, 2, ni->last_sent_by_id);
    if (ni->interval_secs) w_varint(&w, 3, ni->interval_secs);
    for (int i = 0; i < ni->n; i++) {
        uint8_t sub[24];
        pbw_t s = {sub, sizeof(sub), 0, false};
        w_varint(&s, 1, ni->nb[i].node_id);
        w_float(&s, 2, ni->nb[i].snr);
        if (ni->nb[i].last_rx_time) w_fixed32(&s, 3, ni->nb[i].last_rx_time);
        if (ni->nb[i].interval_secs) w_varint(&s, 4, ni->nb[i].interval_secs);
        w_sub(&w, 4, sub, s.len, !s.overflow);
    }
    return w.overflow ? 0 : w.len;
}

/* ---- Waypoint ------------------------------------------------------------- */

bool mt_waypoint_decode(const uint8_t *buf, size_t n, mt_waypoint_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        switch (f.field) {
        case 1: if (f.wt == WT_VARINT) out->id = (uint32_t)f.v; break;
        case 2: if (f.wt == WT_I32) { out->has_lat = true; out->lat_i = (int32_t)(uint32_t)f.v; } break;
        case 3: if (f.wt == WT_I32) { out->has_lon = true; out->lon_i = (int32_t)(uint32_t)f.v; } break;
        case 4: if (f.wt == WT_VARINT) out->expire = (uint32_t)f.v; break;
        case 5: if (f.wt == WT_VARINT) out->locked_to = (uint32_t)f.v; break;
        case 6: if (f.wt == WT_LEN) copy_str(out->name, sizeof(out->name), &f); break;
        case 7: if (f.wt == WT_LEN) copy_str(out->description, sizeof(out->description), &f); break;
        case 8: if (f.wt == WT_I32) out->icon = (uint32_t)f.v; break;
        default: break;
        }
    }
    return rc == 0;
}

size_t mt_waypoint_encode(const mt_waypoint_t *wp, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_varint(&w, 1, wp->id);
    if (wp->has_lat) w_fixed32(&w, 2, (uint32_t)wp->lat_i);
    if (wp->has_lon) w_fixed32(&w, 3, (uint32_t)wp->lon_i);
    if (wp->expire) w_varint(&w, 4, wp->expire);
    if (wp->locked_to) w_varint(&w, 5, wp->locked_to);
    w_string(&w, 6, wp->name);
    w_string(&w, 7, wp->description);
    if (wp->icon) w_fixed32(&w, 8, wp->icon);
    return w.overflow ? 0 : w.len;
}

/* ---- Paxcount -------------------------------------------------------------- */

size_t mt_paxcount_encode(uint32_t wifi, uint32_t ble, uint32_t uptime, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    if (wifi) w_varint(&w, 1, wifi);
    if (ble) w_varint(&w, 2, ble);
    if (uptime) w_varint(&w, 3, uptime);
    return w.overflow ? 0 : w.len;
}

bool mt_paxcount_decode(const uint8_t *buf, size_t n, uint32_t *wifi, uint32_t *ble, uint32_t *uptime)
{
    *wifi = *ble = *uptime = 0;
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.wt != WT_VARINT) continue;
        if (f.field == 1) *wifi = (uint32_t)f.v;
        if (f.field == 2) *ble = (uint32_t)f.v;
        if (f.field == 3) *uptime = (uint32_t)f.v;
    }
    return rc == 0;
}

/* ---- StoreAndForward ------------------------------------------------------ */

bool mt_sf_decode(const uint8_t *buf, size_t n, mt_sf_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.field == 1 && f.wt == WT_VARINT) out->rr = (uint32_t)f.v;
        if (f.field == 3 && f.wt == WT_LEN) {
            out->has_history = true;
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.wt != WT_VARINT) continue;
                if (g.field == 1) out->history_messages = (uint32_t)g.v;
                if (g.field == 2) out->window = (uint32_t)g.v;
                if (g.field == 3) out->last_request = (uint32_t)g.v;
            }
        }
        if (f.field == 4 && f.wt == WT_LEN) {
            out->has_heartbeat = true;
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.wt != WT_VARINT) continue;
                if (g.field == 1) out->hb_period = (uint32_t)g.v;
                if (g.field == 2) out->hb_secondary = (uint32_t)g.v;
            }
        }
        if (f.field == 5 && f.wt == WT_LEN) {
            out->text = f.data;
            out->text_len = f.len;
        }
    }
    return rc == 0;
}

size_t mt_sf_encode(const mt_sf_t *sf, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_varint(&w, 1, sf->rr);
    if (sf->has_history) {
        uint8_t sub[20];
        pbw_t s = {sub, sizeof(sub), 0, false};
        if (sf->history_messages) w_varint(&s, 1, sf->history_messages);
        if (sf->window) w_varint(&s, 2, sf->window);
        if (sf->last_request) w_varint(&s, 3, sf->last_request);
        w_sub(&w, 3, sub, s.len, !s.overflow);
    }
    return w.overflow ? 0 : w.len;
}

/* ---- ChannelSet ------------------------------------------------------------ */

bool mt_channelset_decode(const uint8_t *buf, size_t n, mt_channelset_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.field == 1 && f.wt == WT_LEN && out->n < MT_MAX_CHANNELS) {
            mt_chset_ch_t *c = &out->ch[out->n++];
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.field == 2 && g.wt == WT_LEN) {
                    c->psk_len = (uint8_t)(g.len > 32 ? 32 : g.len);
                    memcpy(c->psk, g.data, c->psk_len);
                }
                if (g.field == 3 && g.wt == WT_LEN) copy_str(c->name, sizeof(c->name), &g);
                if (g.field == 5 && g.wt == WT_VARINT) c->uplink = g.v != 0;
                if (g.field == 6 && g.wt == WT_VARINT) c->downlink = g.v != 0;
                if (g.field == 7 && g.wt == WT_LEN) {
                    pbr_t m = {g.data, g.len, 0};
                    pbf_t h;
                    while (next_field(&m, &h) == 1) {
                        if (h.field == 1 && h.wt == WT_VARINT) c->position_precision = (uint32_t)h.v;
                    }
                }
            }
        }
        if (f.field == 2 && f.wt == WT_LEN) {
            out->has_lora = true;
            pbr_t s = {f.data, f.len, 0};
            pbf_t g;
            while (next_field(&s, &g) == 1) {
                if (g.wt != WT_VARINT) continue;
                switch (g.field) {
                case 1: out->use_preset = g.v != 0; break;
                case 2: out->preset = (uint32_t)g.v; break;
                case 7: out->region = (uint32_t)g.v; break;
                case 8: out->hop_limit = (uint32_t)g.v; break;
                case 9: out->tx_enabled = g.v != 0; break;
                case 10: out->tx_power = (int32_t)g.v; break;
                case 11: out->channel_num = (uint32_t)g.v; break;
                default: break;
                }
            }
        }
    }
    return rc == 0 && out->n > 0;
}

size_t mt_channelset_encode(const mt_channelset_t *cs, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    for (int i = 0; i < cs->n; i++) {
        const mt_chset_ch_t *c = &cs->ch[i];
        uint8_t sub[64], mod[8];
        pbw_t m = {mod, sizeof(mod), 0, false};
        if (c->position_precision) w_varint(&m, 1, c->position_precision);
        pbw_t s = {sub, sizeof(sub), 0, false};
        w_bytes(&s, 2, c->psk, c->psk_len);
        w_string(&s, 3, c->name);
        if (c->uplink) w_varint(&s, 5, 1);
        if (c->downlink) w_varint(&s, 6, 1);
        if (m.len) w_sub(&s, 7, mod, m.len, !m.overflow);
        w_sub(&w, 1, sub, s.len, !s.overflow);
    }
    if (cs->has_lora) {
        uint8_t sub[40];
        pbw_t s = {sub, sizeof(sub), 0, false};
        if (cs->use_preset) w_varint(&s, 1, 1);
        if (cs->preset) w_varint(&s, 2, cs->preset);
        if (cs->region) w_varint(&s, 7, cs->region);
        if (cs->hop_limit) w_varint(&s, 8, cs->hop_limit);
        if (cs->tx_enabled) w_varint(&s, 9, 1);
        if (cs->tx_power) w_varint(&s, 10, (uint64_t)(int64_t)cs->tx_power);
        if (cs->channel_num) w_varint(&s, 11, cs->channel_num);
        w_sub(&w, 2, sub, s.len, !s.overflow);
    }
    return w.overflow ? 0 : w.len;
}

/* ---- Routing --------------------------------------------------------------- */

bool mt_routing_decode(const uint8_t *buf, size_t n, mt_routing_t *out)
{
    memset(out, 0, sizeof(*out));
    pbr_t r = {buf, n, 0};
    pbf_t f;
    int rc;
    while ((rc = next_field(&r, &f)) == 1) {
        if (f.field == 3 && f.wt == WT_VARINT) {
            out->has_error = true;
            out->error_reason = (uint32_t)f.v;
        }
    }
    return rc == 0;
}

size_t mt_routing_encode_error(uint32_t reason, uint8_t *buf, size_t cap)
{
    pbw_t w = {buf, cap, 0, false};
    w_varint(&w, 3, reason); /* oneof member: encoded even when 0 */
    return w.overflow ? 0 : w.len;
}
