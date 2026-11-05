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
        case 8: if (f.wt == WT_LEN) out->has_public_key = f.len > 0; break;
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
        default: break;
        }
    }
    return rc == 0;
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
