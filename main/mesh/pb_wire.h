// Tiny protobuf wire helpers shared by the admin, store & forward and MQTT
// codecs. Header-only so each file gets its own static copy. Pure C.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
} pw_t;

static inline void pw_init(pw_t *w, uint8_t *buf, size_t cap)
{
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->overflow = false;
}

static inline void pw_put(pw_t *w, uint8_t b)
{
    if (w->len >= w->cap) {
        w->overflow = true;
        return;
    }
    w->buf[w->len++] = b;
}

static inline void pw_raw_varint(pw_t *w, uint64_t v)
{
    do {
        uint8_t b = v & 0x7f;
        v >>= 7;
        pw_put(w, v ? (uint8_t)(b | 0x80) : b);
    } while (v);
}

static inline void pw_tag(pw_t *w, uint32_t field, uint8_t wt)
{
    pw_raw_varint(w, ((uint64_t)field << 3) | wt);
}

/** Varint field; skipped when zero (proto3 default). */
static inline void pw_u(pw_t *w, uint32_t field, uint64_t v)
{
    if (!v) return;
    pw_tag(w, field, 0);
    pw_raw_varint(w, v);
}

/** Varint field written even when zero (oneof members). */
static inline void pw_u_always(pw_t *w, uint32_t field, uint64_t v)
{
    pw_tag(w, field, 0);
    pw_raw_varint(w, v);
}

static inline void pw_i32(pw_t *w, uint32_t field, int32_t v)
{
    if (!v) return;
    pw_tag(w, field, 0);
    pw_raw_varint(w, (uint64_t)(int64_t)v);
}

static inline void pw_bool(pw_t *w, uint32_t field, bool v)
{
    if (v) pw_u(w, field, 1);
}

static inline void pw_fixed32(pw_t *w, uint32_t field, uint32_t v)
{
    pw_tag(w, field, 5);
    for (int i = 0; i < 4; i++) pw_put(w, (uint8_t)(v >> (8 * i)));
}

static inline void pw_float(pw_t *w, uint32_t field, float f)
{
    if (f == 0.0f) return;
    uint32_t v;
    memcpy(&v, &f, 4);
    pw_fixed32(w, field, v);
}

static inline void pw_bytes(pw_t *w, uint32_t field, const void *data, size_t n)
{
    pw_tag(w, field, 2);
    pw_raw_varint(w, n);
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < n; i++) pw_put(w, p[i]);
}

static inline void pw_str(pw_t *w, uint32_t field, const char *s)
{
    if (s && s[0]) pw_bytes(w, field, s, strlen(s));
}

/* Nested message: reserve a 2-byte length (sub-messages here are < 16 KB), fill, patch. */
typedef struct {
    size_t at;
} pw_mark_t;

static inline pw_mark_t pw_begin(pw_t *w, uint32_t field)
{
    pw_tag(w, field, 2);
    pw_mark_t m = {w->len};
    pw_put(w, 0x80);
    pw_put(w, 0x00);
    return m;
}

static inline void pw_end(pw_t *w, pw_mark_t m)
{
    if (w->overflow) return;
    size_t n = w->len - m.at - 2;
    if (n < 0x80) {
        /* shrink to a 1-byte length */
        w->buf[m.at] = (uint8_t)n;
        memmove(&w->buf[m.at + 1], &w->buf[m.at + 2], n);
        w->len--;
    } else {
        w->buf[m.at] = (uint8_t)(0x80 | (n & 0x7f));
        w->buf[m.at + 1] = (uint8_t)(n >> 7);
    }
}

/* ---- Reader ------------------------------------------------------------------------- */

typedef struct {
    const uint8_t *p;
    size_t n;
    size_t pos;
} pr_t;

typedef struct {
    uint32_t field;
    uint8_t wt;
    uint64_t v;
    const uint8_t *data;
    size_t len;
} pf_t;

static inline void pr_init(pr_t *r, const uint8_t *p, size_t n)
{
    r->p = p;
    r->n = n;
    r->pos = 0;
}

static inline bool pr_varint(pr_t *r, uint64_t *v)
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

/** 1 = field read, 0 = end, -1 = malformed. */
static inline int pr_next(pr_t *r, pf_t *f)
{
    if (r->pos >= r->n) return 0;
    uint64_t tag;
    if (!pr_varint(r, &tag)) return -1;
    f->field = (uint32_t)(tag >> 3);
    f->wt = tag & 7;
    f->data = NULL;
    f->len = 0;
    if (f->field == 0) return -1;
    switch (f->wt) {
    case 0:
        return pr_varint(r, &f->v) ? 1 : -1;
    case 1:
        if (r->pos + 8 > r->n) return -1;
        f->v = 0;
        for (int i = 0; i < 8; i++) f->v |= (uint64_t)r->p[r->pos + i] << (8 * i);
        r->pos += 8;
        return 1;
    case 5:
        if (r->pos + 4 > r->n) return -1;
        f->v = 0;
        for (int i = 0; i < 4; i++) f->v |= (uint64_t)r->p[r->pos + i] << (8 * i);
        r->pos += 4;
        return 1;
    case 2: {
        uint64_t len;
        if (!pr_varint(r, &len) || len > r->n - r->pos) return -1;
        f->data = &r->p[r->pos];
        f->len = (size_t)len;
        r->pos += (size_t)len;
        return 1;
    }
    default:
        return -1;
    }
}

static inline float pf_float(const pf_t *f)
{
    uint32_t v = (uint32_t)f->v;
    float x;
    memcpy(&x, &v, 4);
    return x;
}

static inline void pf_str(const pf_t *f, char *out, size_t cap)
{
    if (!cap) return;
    size_t n = f->wt == 2 ? (f->len < cap - 1 ? f->len : cap - 1) : 0;
    if (n) memcpy(out, f->data, n);
    out[n] = 0;
}
