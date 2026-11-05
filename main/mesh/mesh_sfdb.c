#include "mesh_sfdb.h"

#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "sfdb";

#define REC_SIZE   256
#define SECT_SIZE  4096
#define PER_SECT   (SECT_SIZE / REC_SIZE)
#define MAGIC      0x31424653u /* "SFB1" */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t time;
    uint32_t from, to, id, reply_id, emoji;
    uint8_t channel;
    uint8_t len;
    uint8_t text[SFDB_TEXT_MAX];
    uint8_t pad[REC_SIZE - 34 - SFDB_TEXT_MAX - 4];
    uint32_t crc;
} disk_t;

_Static_assert(sizeof(disk_t) == REC_SIZE, "record size");

static const esp_partition_t *s_part;
static SemaphoreHandle_t s_lock;
static uint32_t s_slots, s_head /* next seq */, s_count;

static uint32_t rec_crc(const disk_t *d)
{
    return esp_rom_crc32_le(0, (const uint8_t *)d, offsetof(disk_t, crc));
}

static bool read_slot(uint32_t slot, disk_t *d)
{
    if (esp_partition_read(s_part, (size_t)slot * REC_SIZE, d, sizeof(*d)) != ESP_OK) return false;
    return d->magic == MAGIC && d->crc == rec_crc(d);
}

esp_err_t sfdb_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x42, "sfdb");
    if (!s_part) {
        ESP_LOGW(TAG, "no sfdb partition: store & forward server unavailable");
        return ESP_ERR_NOT_FOUND;
    }
    s_slots = (uint32_t)(s_part->size / REC_SIZE);
    uint32_t max_seq = 0, count = 0;
    struct {
        uint32_t magic, seq;
    } h;
    for (uint32_t i = 0; i < s_slots; i++) {
        if (esp_partition_read(s_part, (size_t)i * REC_SIZE, &h, sizeof(h)) != ESP_OK) continue;
        if (h.magic != MAGIC) continue;
        count++;
        if (h.seq > max_seq) max_seq = h.seq;
    }
    s_head = max_seq + 1;
    s_count = count > s_slots ? s_slots : count;
    ESP_LOGI(TAG, "%lu of %lu history slots used", (unsigned long)s_count, (unsigned long)s_slots);
    return ESP_OK;
}

bool sfdb_ready(void) { return s_part != NULL; }
uint32_t sfdb_capacity(void) { return s_part ? s_slots - PER_SECT : 0; } /* one sector is always being recycled */
uint32_t sfdb_count(void) { return s_count; }
uint32_t sfdb_total(void) { return s_head ? s_head - 1 : 0; }

esp_err_t sfdb_add(sfdb_rec_t *r)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t seq = s_head;
    uint32_t slot = seq % s_slots;
    esp_err_t err = ESP_OK;
    if (slot % PER_SECT == 0) {
        /* entering a sector: recycle it (drops the 16 oldest) */
        uint32_t lost = 0;
        disk_t d;
        for (uint32_t i = 0; i < PER_SECT; i++) {
            if (esp_partition_read(s_part, (size_t)(slot + i) * REC_SIZE, &d, 8) == ESP_OK && d.magic == MAGIC) lost++;
        }
        err = esp_partition_erase_range(s_part, (size_t)slot * REC_SIZE, SECT_SIZE);
        s_count = s_count > lost ? s_count - lost : 0;
    }
    if (err == ESP_OK) {
        disk_t d;
        memset(&d, 0xff, sizeof(d));
        d.magic = MAGIC;
        d.seq = seq;
        d.time = r->time;
        d.from = r->from;
        d.to = r->to;
        d.id = r->id;
        d.reply_id = r->reply_id;
        d.emoji = r->emoji;
        d.channel = r->channel;
        d.len = r->len > SFDB_TEXT_MAX ? SFDB_TEXT_MAX : r->len;
        memcpy(d.text, r->text, d.len);
        d.crc = rec_crc(&d);
        err = esp_partition_write(s_part, (size_t)slot * REC_SIZE, &d, sizeof(d));
        if (err == ESP_OK) {
            r->seq = seq;
            s_head = seq + 1;
            if (s_count < s_slots) s_count++;
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

bool sfdb_read(uint32_t seq, sfdb_rec_t *out)
{
    if (!s_part || !seq || seq >= s_head || s_head - seq > s_slots) return false;
    disk_t d;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = read_slot(seq % s_slots, &d) && d.seq == seq;
    xSemaphoreGive(s_lock);
    if (!ok) return false;
    out->seq = d.seq;
    out->time = d.time;
    out->from = d.from;
    out->to = d.to;
    out->id = d.id;
    out->reply_id = d.reply_id;
    out->emoji = d.emoji;
    out->channel = d.channel;
    out->len = d.len;
    memcpy(out->text, d.text, d.len);
    return true;
}

int sfdb_select(uint32_t since_s, uint32_t dest, uint8_t ch, uint32_t *seqs, int max)
{
    if (!s_part) return 0;
    int n = 0;
    uint32_t first = s_head > s_slots ? s_head - s_slots : 1;
    /* header fields only: magic, seq, time, from, to (20 bytes) and the channel byte */
    struct __attribute__((packed)) {
        uint32_t magic, seq, time, from, to, id, reply, emoji;
        uint8_t channel;
    } h;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (uint32_t seq = first; seq < s_head; seq++) {
        if (esp_partition_read(s_part, (size_t)(seq % s_slots) * REC_SIZE, &h, sizeof(h)) != ESP_OK) continue;
        if (h.magic != MAGIC || h.seq != seq) continue;
        if (h.time && h.time <= since_s) continue;
        if (h.from == dest || h.channel != ch) continue;
        if (h.to != 0xffffffffu && h.to != dest) continue;
        if (max && n < max) seqs[n] = seq;
        n++;
    }
    xSemaphoreGive(s_lock);
    /* keep the newest `max` when there are more */
    if (max && n > max) {
        int skip = n - max;
        int k = 0;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (uint32_t seq = first; seq < s_head && k < max; seq++) {
            if (esp_partition_read(s_part, (size_t)(seq % s_slots) * REC_SIZE, &h, sizeof(h)) != ESP_OK) continue;
            if (h.magic != MAGIC || h.seq != seq) continue;
            if (h.time && h.time <= since_s) continue;
            if (h.from == dest || h.channel != ch) continue;
            if (h.to != 0xffffffffu && h.to != dest) continue;
            if (skip > 0) {
                skip--;
                continue;
            }
            seqs[k++] = seq;
        }
        xSemaphoreGive(s_lock);
        return k;
    }
    return n;
}

esp_err_t sfdb_erase(void)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = esp_partition_erase_range(s_part, 0, s_part->size);
    s_head = 1;
    s_count = 0;
    xSemaphoreGive(s_lock);
    return err;
}
