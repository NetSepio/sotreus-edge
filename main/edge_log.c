#include "edge_log.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
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
static SemaphoreHandle_t s_route;   /* orders sequence numbers with where each record goes */
static uint32_t s_floor_saved;      /* sequence numbers below this are safe from reuse after a restart */
static edge_log_hook_t s_hook;
static uint32_t s_cut;              /* confirmed by the phone and deleted: seq <= s_cut is never served */
static volatile bool s_releasing;
static uint32_t s_released;         /* records erased after confirmation since boot */
#define FLOOR_STEP 1024

/* Records lost at restarts. RTC memory survives a software restart, a crash and a watchdog reset,
 * but not a power loss or the reset pin (flashing uses it); the CRC tells a kept block from
 * leftover bits. Every seq above the largest of `safe`, the newest stored seq and the confirmed
 * cut, up to `last`, was numbered but neither stored nor confirmed when the restart came. */
#define RL_MAGIC 0x524c3031u /* "RL01" */
typedef struct {
    uint32_t magic;
    uint32_t since_boot;  /* counting runs from this boot; the restart into it is not known */
    uint32_t boot;        /* the boot that `last` and `safe` belong to */
    uint32_t last;        /* last seq numbered in `boot` */
    uint32_t safe;        /* seqs <= safe were confirmed by the phone or erased on purpose */
    uint32_t total;
    uint32_t n;
    edge_log_lost_t r[EDGE_LOG_LOST_MAX]; /* newest last */
    uint32_t crc;
} restart_loss_t;
static RTC_NOINIT_ATTR restart_loss_t s_rl;

static uint32_t rl_crc(void)
{
    return esp_rom_crc32_le(0, (const uint8_t *)&s_rl, offsetof(restart_loss_t, crc));
}

static void rl_seal(void)
{
    s_rl.crc = rl_crc();
}

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

static uint32_t edge_cut_load(void)
{
    return edge_log_cut_get();
}

/* Runs once at boot, after the head and the confirmed cut are known. */
static void restart_loss_init(void)
{
    uint32_t boot = edge_identity()->boot_count;
    bool kept = s_rl.magic == RL_MAGIC && s_rl.crc == rl_crc() && s_rl.n <= EDGE_LOG_LOST_MAX &&
                s_rl.since_boot <= s_rl.boot && s_rl.boot < boot;
    if (kept) {
        uint32_t base = s_rl.safe;
        if (s_head_seq > base) base = s_head_seq;
        if (s_cut > base) base = s_cut;
        if (s_rl.last > base) {
            if (s_rl.n == EDGE_LOG_LOST_MAX) {
                memmove(&s_rl.r[0], &s_rl.r[1], (EDGE_LOG_LOST_MAX - 1) * sizeof(s_rl.r[0]));
                s_rl.n--;
            }
            s_rl.r[s_rl.n++] = (edge_log_lost_t){.boot = s_rl.boot, .from = base + 1, .to = s_rl.last};
            s_rl.total += s_rl.last - base;
            ESP_LOGW(TAG, "restart lost %lu record(s): seq %lu-%lu of boot %lu were neither stored nor confirmed",
                     (unsigned long)(s_rl.last - base), (unsigned long)(base + 1), (unsigned long)s_rl.last,
                     (unsigned long)s_rl.boot);
        } else {
            ESP_LOGI(TAG, "restart lost no records (counting since boot %lu)", (unsigned long)s_rl.since_boot);
        }
    } else {
        memset(&s_rl, 0, sizeof(s_rl));
        s_rl.magic = RL_MAGIC;
        s_rl.since_boot = boot;
        ESP_LOGI(TAG, "records lost at this restart: not known (RTC memory not kept); counting from boot %lu",
                 (unsigned long)boot);
    }
    s_rl.boot = boot;
    s_rl.last = s_rl.safe = s_next_seq - 1;
    rl_seal();
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
    s_route = xSemaphoreCreateMutex();
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
    s_floor_saved = floor;
    s_cut = edge_cut_load();
    if (s_next_seq == 0) s_next_seq = 1;

    ESP_LOGI(TAG, "%lu slots, head seq %lu, oldest %lu, next %lu", (unsigned long)s_cap,
             (unsigned long)s_head_seq, (unsigned long)oldest_locked(), (unsigned long)s_next_seq);
    restart_loss_init();
    return ESP_OK;
}

static void prepare(edge_rec_t *rec)
{
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
}

/* Every record gets the next sequence number whether it is stored or handed to the phone.
 * The floor in NVS runs ahead, so numbers handed to the phone are never reused after a restart. */
static void assign_seq(edge_rec_t *rec)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    rec->seq = s_next_seq++;
    s_rl.last = rec->seq;
    rl_seal();
    bool save = s_next_seq >= s_floor_saved;
    if (save) s_floor_saved = s_next_seq + FLOOR_STEP;
    uint32_t floor = s_floor_saved;
    xSemaphoreGive(s_lock);
    if (save) edge_seq_floor_set(floor);
    rec->crc = rec_crc(rec);
}

/* Writes a record that already has its sequence number (always higher than any stored one). */
static esp_err_t write_slot(edge_rec_t *rec)
{
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
    if (err == ESP_OK) err = esp_partition_write(s_part, (size_t)slot * EDGE_REC_SIZE, rec, EDGE_REC_SIZE);
    if (err == ESP_OK) {
        if (slot % PER_SECT == 0) s_first[sect] = rec->seq;
        s_head_slot = (int32_t)slot;
        s_head_seq = rec->seq;
        s_written++;
    } else {
        s_failed++;
        ESP_LOGW(TAG, "write failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t edge_log_append(edge_rec_t *rec)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    if (s_erasing) return ESP_ERR_INVALID_STATE;
    prepare(rec);
    xSemaphoreTake(s_route, portMAX_DELAY);
    assign_seq(rec);
    esp_err_t err = ESP_OK;
    if (!(s_hook && s_hook(rec))) err = write_slot(rec);
    xSemaphoreGive(s_route);
    return err;
}

void edge_log_set_hook(edge_log_hook_t hook)
{
    s_hook = hook;
}

void edge_log_route_lock(void)
{
    xSemaphoreTake(s_route, portMAX_DELAY);
}

void edge_log_route_unlock(void)
{
    xSemaphoreGive(s_route);
}

esp_err_t edge_log_write_assigned(edge_rec_t *rec)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    return write_slot(rec);
}

uint32_t edge_log_next_seq(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t n = s_next_seq;
    xSemaphoreGive(s_lock);
    return n;
}

int edge_log_read_from(uint32_t from_seq, edge_rec_t *out, int max)
{
    if (!s_part || max <= 0 || s_erasing) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = 0;
    if (from_seq <= s_cut) from_seq = s_cut + 1;
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
    if (out->oldest_seq && out->oldest_seq <= s_cut) out->oldest_seq = s_cut + 1;
    if (out->oldest_seq > s_head_seq) out->oldest_seq = 0;
    /* Count by sector: sequence numbers have gaps (records handed to the phone, the restart floor). */
    uint32_t head_sect = s_head_slot >= 0 ? (uint32_t)s_head_slot / PER_SECT : UINT32_MAX;
    out->count = 0;
    for (uint32_t s = 0; out->oldest_seq && s < s_nsect; s++) {
        if (!s_first[s]) continue;
        out->count += s == head_sect ? (uint32_t)s_head_slot % PER_SECT + 1 : PER_SECT;
    }
    out->released = s_released;
    out->releasing = s_releasing;
    if (out->count > s_cap) out->count = s_cap;
    out->capacity = s_cap;
    out->written = s_written;
    out->failed = s_failed;
    out->erasing = s_erasing;
    xSemaphoreGive(s_lock);
}

void edge_log_restart_loss(edge_log_restart_loss_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_part) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->since_boot = s_rl.since_boot;
    out->total = s_rl.total;
    out->n = (int)s_rl.n;
    memcpy(out->r, s_rl.r, sizeof(out->r));
    xSemaphoreGive(s_lock);
}

void edge_log_delivered(uint32_t seq)
{
    if (!s_part) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (seq > s_rl.safe && seq <= s_rl.last) {
        s_rl.safe = seq;
        rl_seal();
    }
    xSemaphoreGive(s_lock);
}

/* ---- Delete after a confirmed sync ------------------------------------------------- */

/* Highest seq stored in a sector (record headers only), 0 if none. */
static uint32_t sector_max_seq(uint32_t sect)
{
    uint32_t mx = 0;
    for (uint32_t k = 0; k < PER_SECT; k++) {
        uint32_t seq;
        if (esp_partition_read(s_part, ((size_t)sect * PER_SECT + k) * EDGE_REC_SIZE, &seq, sizeof(seq)) != ESP_OK) continue;
        if (seq != 0xffffffffu && seq <= s_head_seq && seq > mx) mx = seq;
    }
    return mx;
}

/* Erases every sector whose records all have seq <= through. When that includes the sector being
 * written, writing moves on to the next sector so every sector still starts with a record. */
static void do_release(uint32_t through)
{
    for (uint32_t s = 0; s < s_nsect; s++) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t head_sect = s_head_slot >= 0 ? (uint32_t)s_head_slot / PER_SECT : UINT32_MAX;
        uint32_t f = s_first[s];
        bool erase = f && f <= through && sector_max_seq(s) <= through;
        if (erase) {
            uint32_t n = s == head_sect ? (uint32_t)s_head_slot % PER_SECT + 1 : PER_SECT;
            if (esp_partition_erase_range(s_part, (size_t)s * SECT_SIZE, SECT_SIZE) == ESP_OK) {
                s_first[s] = 0;
                s_released += n;
                if (s == head_sect) s_head_slot = (int32_t)(s * PER_SECT + PER_SECT - 1); /* next write: next sector */
            }
        }
        xSemaphoreGive(s_lock);
        if (erase) vTaskDelay(1);
    }
    s_releasing = false;
    ESP_LOGI(TAG, "released records up to seq %lu (%lu erased since boot)", (unsigned long)through, (unsigned long)s_released);
}

static void release_task(void *arg)
{
    do_release((uint32_t)(uintptr_t)arg);
    vTaskDelete(NULL);
}

esp_err_t edge_log_release(uint32_t through, uint32_t *applied)
{
    if (!s_part) return ESP_ERR_INVALID_STATE;
    if (s_erasing || s_releasing) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (through > s_head_seq) through = s_head_seq;
    bool more = through > s_cut;
    if (more) s_cut = through;
    xSemaphoreGive(s_lock);
    if (applied) *applied = through;
    if (!more) return ESP_OK;
    edge_log_cut_set(through); /* served no more, even before the erase finishes */
    s_releasing = true;
    if (xTaskCreate(release_task, "log_release", 3072, (void *)(uintptr_t)through, 3, NULL) != pdPASS) {
        s_releasing = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void do_erase(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Never reuse sequence numbers on this device. */
    s_floor_saved = s_next_seq + FLOOR_STEP;
    edge_seq_floor_set(s_floor_saved);
    /* Stored records erased on purpose are not lost at a later restart. */
    if (s_head_seq > s_rl.safe) {
        s_rl.safe = s_head_seq;
        rl_seal();
    }
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
