#include "edge_log.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "edge_config.h"
#include "edge_time.h"

static const char *TAG = "log";

#define SECT_SIZE   4096
#define PER_SECT    (SECT_SIZE / EDGE_REC_SIZE)
#define CRC_LEN     offsetof(edge_rec_t, crc)

static const esp_partition_t *s_part;
static SemaphoreHandle_t s_lock;
static uint32_t s_nsect;
static uint32_t s_cap;
static uint32_t *s_first;     /* first seq stored in each sector, 0 = none */
static int32_t s_head_slot = -1;
static uint32_t s_head_seq;
static uint32_t s_next_seq = 1;
static volatile bool s_erasing;
static uint32_t s_written, s_failed;

static uint32_t rec_crc(const edge_rec_t *r)
{
    return esp_rom_crc32_le(0, (const uint8_t *)r, CRC_LEN);
}

static bool rec_valid(const edge_rec_t *r)
{
    if (r->seq == 0 || r->seq == 0xffffffffu) return false;
    if (r->type == 0 || r->type == 0xff) return false;
    if (r->body_len > EDGE_REC_BODY_MAX) return false;
    return r->crc == rec_crc(r);
}

static esp_err_t read_slot(uint32_t slot, edge_rec_t *r)
{
    return esp_partition_read(s_part, (size_t)slot * EDGE_REC_SIZE, r, EDGE_REC_SIZE);
}

static bool slot_blank(uint32_t slot)
{
    uint32_t words[8];
    if (esp_partition_read(s_part, (size_t)slot * EDGE_REC_SIZE, words, sizeof(words)) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < 8; i++) {
        if (words[i] != 0xffffffffu) return false;
    }
    return true;
}

static uint32_t oldest_locked(void)
{
    uint32_t oldest = 0;
    for (uint32_t s = 0; s < s_nsect; s++) {
        uint32_t f = s_first[s];
        if (f && (oldest == 0 || f < oldest)) oldest = f;
    }
    return oldest;
}

esp_err_t edge_log_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "obslog");
    if (!s_part) {
        ESP_LOGE(TAG, "obslog partition missing");
        return ESP_ERR_NOT_FOUND;
    }
    s_nsect = s_part->size / SECT_SIZE;
    s_cap = s_nsect * PER_SECT;
    s_first = calloc(s_nsect, sizeof(uint32_t));
    if (!s_first) return ESP_ERR_NO_MEM;

    edge_rec_t *r = malloc(sizeof(edge_rec_t));
    if (!r) return ESP_ERR_NO_MEM;

    uint32_t max_seq = 0;
    int32_t max_sect = -1;
    for (uint32_t s = 0; s < s_nsect; s++) {
        if (read_slot(s * PER_SECT, r) == ESP_OK && rec_valid(r)) {
            s_first[s] = r->seq;
            if (r->seq > max_seq) {
                max_seq = r->seq;
                max_sect = (int32_t)s;
            }
        }
    }

    if (max_sect >= 0) {
        uint32_t slot = (uint32_t)max_sect * PER_SECT;
        uint32_t prev = max_seq;
        s_head_slot = (int32_t)slot;
        for (uint32_t k = 1; k < PER_SECT; k++) {
            if (read_slot(slot + k, r) != ESP_OK || !rec_valid(r) || r->seq <= prev) break;
            prev = r->seq;
            s_head_slot = (int32_t)(slot + k);
        }
        s_head_seq = prev;
    }
    free(r);

    uint32_t floor = edge_seq_floor_get();
    s_next_seq = s_head_seq + 1;
    if (s_next_seq < floor) s_next_seq = floor;
    if (s_next_seq == 0) s_next_seq = 1;

    ESP_LOGI(TAG, "%lu slots, head seq %lu, oldest %lu, next %lu", (unsigned long)s_cap,
             (unsigned long)s_head_seq, (unsigned long)oldest_locked(), (unsigned long)s_next_seq);
    return ESP_OK;
}

esp_err_t edge_log_append(edge_rec_t *rec)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    if (s_erasing) return ESP_ERR_INVALID_STATE;

    /* Convert times. */
    int64_t f = edge_time_to_unix_ms(rec->first_ms);
    int64_t l = edge_time_to_unix_ms(rec->last_ms);
    rec->flags &= (uint8_t)~EDGE_RECF_TIME_VALID;
    if (f >= 0 && l >= 0) {
        rec->first_ms = f;
        rec->last_ms = l;
        rec->flags |= EDGE_RECF_TIME_VALID;
    }
    rec->boot = (uint16_t)edge_identity()->boot_count;
    rec->format = EDGE_REC_FORMAT;
    if (rec->body_len > EDGE_REC_BODY_MAX) rec->body_len = EDGE_REC_BODY_MAX;
    if (rec->body_len < EDGE_REC_BODY_MAX) {
        memset(rec->body + rec->body_len, 0, EDGE_REC_BODY_MAX - rec->body_len);
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_erasing) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t slot = (uint32_t)((s_head_slot + 1) % (int32_t)s_cap);
    if (slot % PER_SECT != 0 && !slot_blank(slot)) {
        /* Partial write from a power loss: start fresh in the next sector. */
        slot = ((slot / PER_SECT + 1) * PER_SECT) % s_cap;
    }
    uint32_t sect = slot / PER_SECT;
    esp_err_t err = ESP_OK;
    if (slot % PER_SECT == 0) {
        err = esp_partition_erase_range(s_part, (size_t)sect * SECT_SIZE, SECT_SIZE);
        s_first[sect] = 0;
    }
    if (err == ESP_OK) {
        rec->seq = s_next_seq;
        rec->crc = rec_crc(rec);
        err = esp_partition_write(s_part, (size_t)slot * EDGE_REC_SIZE, rec, EDGE_REC_SIZE);
    }
    if (err == ESP_OK) {
        if (slot % PER_SECT == 0) s_first[sect] = rec->seq;
        s_head_slot = (int32_t)slot;
        s_head_seq = rec->seq;
        s_next_seq++;
        s_written++;
    } else {
        s_failed++;
        ESP_LOGW(TAG, "write failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_lock);
    return err;
}

int edge_log_read_from(uint32_t from_seq, edge_rec_t *out, int max)
{
    if (!s_part || max <= 0 || s_erasing) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = 0;
    if (s_head_seq == 0 || from_seq > s_head_seq) goto done;
    uint32_t oldest = oldest_locked();
    if (from_seq < oldest) from_seq = oldest;

    int32_t best = -1;
    uint32_t best_first = 0;
    for (uint32_t s = 0; s < s_nsect; s++) {
        uint32_t f = s_first[s];
        if (f && f <= from_seq && f > best_first) {
            best_first = f;
            best = (int32_t)s;
        }
    }
    if (best < 0) goto done;

    uint32_t slot = (uint32_t)best * PER_SECT;
    uint32_t prev = 0;
    for (uint32_t i = 0; i < s_cap && n < max; i++) {
        edge_rec_t *r = &out[n];
        if (read_slot(slot, r) != ESP_OK) break;
        if (!rec_valid(r)) {
            if (slot % PER_SECT == 0) break; /* nothing further */
            /* Gap after a partial write: continue at the next sector. */
            slot = ((slot / PER_SECT + 1) * PER_SECT) % s_cap;
            continue;
        }
        if (r->seq < prev || r->seq > s_head_seq) break; /* wrapped into older data */
        prev = r->seq;
        if (r->seq >= from_seq) n++;
        if (r->seq == s_head_seq) break;
        slot = (slot + 1) % s_cap;
    }
done:
    xSemaphoreGive(s_lock);
    return n;
}

void edge_log_stats(edge_log_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_part) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->head_seq = s_head_seq;
    out->oldest_seq = s_head_seq ? oldest_locked() : 0;
    out->count = s_head_seq && out->oldest_seq ? s_head_seq - out->oldest_seq + 1 : 0;
    if (out->count > s_cap) out->count = s_cap;
    out->capacity = s_cap;
    out->written = s_written;
    out->failed = s_failed;
    out->erasing = s_erasing;
    xSemaphoreGive(s_lock);
}

static void do_erase(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Never reuse sequence numbers on this device. */
    edge_seq_floor_set(s_next_seq);
    xSemaphoreGive(s_lock);

    const size_t block = 64 * 1024;
    for (size_t off = 0; off < s_part->size; off += block) {
        size_t len = s_part->size - off < block ? s_part->size - off : block;
        esp_err_t err = esp_partition_erase_range(s_part, off, len);
        if (err != ESP_OK) ESP_LOGW(TAG, "erase @%u: %s", (unsigned)off, esp_err_to_name(err));
        vTaskDelay(1);
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_first, 0, s_nsect * sizeof(uint32_t));
    s_head_slot = -1;
    s_head_seq = 0;
    s_erasing = false;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "log erased, next seq %lu", (unsigned long)s_next_seq);
}

static void erase_task(void *arg)
{
    do_erase();
    vTaskDelete(NULL);
}

esp_err_t edge_log_erase(bool wait)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    if (s_erasing) return ESP_ERR_INVALID_STATE;
    s_erasing = true;
    if (wait) {
        do_erase();
        return ESP_OK;
    }
    if (xTaskCreate(erase_task, "log_erase", 3072, NULL, 3, NULL) != pdPASS) {
        s_erasing = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
