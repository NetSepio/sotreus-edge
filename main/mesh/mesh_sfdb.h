// Store & forward server history: a ring of 256-byte records on the "sfdb"
// flash partition, so the history survives restarts without any PSRAM.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define SFDB_TEXT_MAX 216

typedef struct {
    uint32_t seq;
    uint32_t time;       /* unix seconds when heard (0 = clock unknown) */
    uint32_t from, to, id, reply_id, emoji;
    uint8_t channel;     /* local channel index */
    uint8_t len;
    uint8_t text[SFDB_TEXT_MAX];
} sfdb_rec_t;

esp_err_t sfdb_init(void);
bool sfdb_ready(void);
uint32_t sfdb_capacity(void);
uint32_t sfdb_count(void);
uint32_t sfdb_total(void);      /* records ever written */
esp_err_t sfdb_add(sfdb_rec_t *r);  /* fills r->seq */
/** Lists sequence numbers of records newer than since_s for dest (broadcast or addressed to it, not from it)
 *  on channel ch, oldest first; returns how many (≤ max). Pass max = 0 to only count. */
int sfdb_select(uint32_t since_s, uint32_t dest, uint8_t ch, uint32_t *seqs, int max);
bool sfdb_read(uint32_t seq, sfdb_rec_t *out);
esp_err_t sfdb_erase(void);
