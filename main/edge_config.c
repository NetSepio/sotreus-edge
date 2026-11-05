#include "edge_config.h"

#include <string.h>

#include "bootloader_random.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "edge_util.h"

static const char *TAG = "config";
static const char *NS = "edge";

static SemaphoreHandle_t s_lock;
static edge_config_t s_cfg;
static edge_identity_t s_id;

static void random_text(char *out, size_t len, const char *alphabet)
{
    size_t n = strlen(alphabet);
    for (size_t i = 0; i < len; i++) out[i] = alphabet[esp_random() % n];
    out[len] = '\0';
}

static void new_pin(char out[9])
{
    random_text(out, 8, "0123456789");
}

void edge_config_defaults(edge_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->version = EDGE_CONFIG_VERSION;
    c->mode = EDGE_MODE_AWARENESS;
    c->link_mode = EDGE_LINK_AUTO;
    c->ble_scan = true;
    c->ble_active = true;
    c->wifi_scan = true;
    c->wifi_probes = true;
    c->probe_ssid_plain = false;
    c->rssi_floor = -100;
    c->close_after_s = 60;
    c->max_window_s = 300;
    c->display_timeout_s = 60;
    c->region = 0; /* unset: LoRa stays silent until the owner picks a region */
    c->preset = 0; /* LONG_FAST */
    c->hop_limit = 3;
    c->rebroadcast = EDGE_REBROADCAST_ALL;
    c->tx_power = 0;
    c->channel_num = 0;
    c->nodeinfo_interval_s = 3 * 3600;
    c->n_channels = 1;
    c->channels[0].name[0] = '\0';
    c->channels[0].psk[0] = 1; /* Meshtastic default key */
    c->channels[0].psk_len = 1;
}

static uint32_t random_node_num(void)
{
    uint32_t n;
    do {
        n = esp_random();
    } while (n < 0x10 || n == 0xffffffffu);
    return n;
}

static void clamp(edge_config_t *c)
{
    if (c->mode > EDGE_MODE_MESH) c->mode = EDGE_MODE_AWARENESS;
    if (c->link_mode > EDGE_LINK_OFF) c->link_mode = EDGE_LINK_AUTO;
    if (c->rssi_floor > -20) c->rssi_floor = -20;
    if (c->close_after_s < 10) c->close_after_s = 10;
    if (c->close_after_s > 3600) c->close_after_s = 3600;
    if (c->max_window_s < c->close_after_s) c->max_window_s = c->close_after_s;
    if (c->max_window_s > 3600) c->max_window_s = 3600;
    if (c->display_timeout_s > 3600) c->display_timeout_s = 3600;
    if (c->hop_limit < 1) c->hop_limit = 1;
    if (c->hop_limit > 7) c->hop_limit = 7;
    if (c->rebroadcast > EDGE_REBROADCAST_NONE) c->rebroadcast = EDGE_REBROADCAST_ALL;
    if (c->tx_power < 0) c->tx_power = 0;
    if (c->tx_power > 30) c->tx_power = 30;
    if (c->nodeinfo_interval_s && c->nodeinfo_interval_s < 600) c->nodeinfo_interval_s = 600;
    if (c->n_channels < 1) c->n_channels = 1;
    if (c->n_channels > EDGE_MESH_MAX_CHANNELS) c->n_channels = EDGE_MESH_MAX_CHANNELS;
    for (int i = 0; i < EDGE_MESH_MAX_CHANNELS; i++) {
        edge_mesh_channel_t *ch = &c->channels[i];
        ch->name[sizeof(ch->name) - 1] = '\0';
        if (ch->psk_len != 0 && ch->psk_len != 1 && ch->psk_len != 16 && ch->psk_len != 32) {
            ch->psk_len = 1;
            ch->psk[0] = 1;
        }
    }
    if (c->node_num < 0x10 || c->node_num == 0xffffffffu) c->node_num = random_node_num();
    c->long_name[sizeof(c->long_name) - 1] = '\0';
    c->short_name[sizeof(c->short_name) - 1] = '\0';
    c->version = EDGE_CONFIG_VERSION;
}

static esp_err_t save_blob(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, key, data, len);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t load_blob(const char *key, void *data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    size_t got = len;
    err = nvs_get_blob(h, key, data, &got);
    nvs_close(h);
    if (err == ESP_OK && got != len) return ESP_ERR_INVALID_SIZE;
    return err;
}

static esp_err_t load_u32(const char *key, uint32_t *v)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    err = nvs_get_u32(h, key, v);
    nvs_close(h);
    return err;
}

static esp_err_t save_u32(const char *key, uint32_t v)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(h, key, v);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static void create_identity(edge_identity_t *id)
{
    memset(id, 0, sizeof(*id));
    /* No radio is running yet, so feed the RNG from the bootloader entropy source. */
    bootloader_random_enable();
    esp_fill_random(id->device_id, sizeof(id->device_id));
    esp_fill_random(id->probe_salt, sizeof(id->probe_salt));
    esp_fill_random(id->sta_mac, sizeof(id->sta_mac));
    id->sta_mac[0] = (uint8_t)((id->sta_mac[0] & 0xfc) | 0x02); /* unicast, locally administered */
    new_pin(id->access_pin);
    random_text(id->suffix, 4, "0123456789ABCDEF");
    random_text(id->ap_password, 12, "abcdefghjkmnpqrstuvwxyz23456789");
    bootloader_random_disable();
    snprintf(id->ap_ssid, sizeof(id->ap_ssid), "Sotreus-Edge-%s", id->suffix);
}

esp_err_t edge_config_init(void)
{
    s_lock = xSemaphoreCreateMutex();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    if (load_blob("id", &s_id, sizeof(s_id)) != ESP_OK) {
        ESP_LOGI(TAG, "first boot: creating device identity");
        create_identity(&s_id);
        s_id.boot_count = 0;
    }
    uint32_t boots = 0;
    load_u32("boots", &boots);
    s_id.boot_count = boots + 1;
    save_u32("boots", s_id.boot_count);
    save_blob("id", &s_id, sizeof(s_id));

    if (load_blob("cfg", &s_cfg, sizeof(s_cfg)) != ESP_OK || s_cfg.version != EDGE_CONFIG_VERSION) {
        ESP_LOGI(TAG, "using default settings");
        bootloader_random_enable();
        edge_config_defaults(&s_cfg);
        s_cfg.node_num = random_node_num();
        bootloader_random_disable();
        snprintf(s_cfg.long_name, sizeof(s_cfg.long_name), "Sotreus Edge %s", s_id.suffix);
        snprintf(s_cfg.short_name, sizeof(s_cfg.short_name), "SE%.2s", s_id.suffix + 2);
        save_blob("cfg", &s_cfg, sizeof(s_cfg));
    }
    clamp(&s_cfg);
    ESP_LOGI(TAG, "boot #%lu, mode=%s", (unsigned long)s_id.boot_count,
             s_cfg.mode == EDGE_MODE_MESH ? "mesh" : "awareness");
    return ESP_OK;
}

void edge_config_get(edge_config_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_cfg;
    xSemaphoreGive(s_lock);
}

esp_err_t edge_config_set(const edge_config_t *in)
{
    edge_config_t c = *in;
    clamp(&c);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = c;
    esp_err_t err = save_blob("cfg", &s_cfg, sizeof(s_cfg));
    xSemaphoreGive(s_lock);
    return err;
}

const edge_identity_t *edge_identity(void)
{
    return &s_id;
}

esp_err_t edge_identity_new_pin(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    new_pin(s_id.access_pin);
    esp_err_t err = save_blob("id", &s_id, sizeof(s_id));
    xSemaphoreGive(s_lock);
    return err;
}

/* ---- Known Wi-Fi networks -------------------------------------------------- */

typedef struct {
    uint8_t count;
    edge_wifi_net_t nets[EDGE_MAX_WIFI_NETS];
} nets_blob_t;

static void nets_load(nets_blob_t *b)
{
    if (load_blob("nets", b, sizeof(*b)) != ESP_OK || b->count > EDGE_MAX_WIFI_NETS) {
        memset(b, 0, sizeof(*b));
    }
}

int edge_wifi_nets_get(edge_wifi_net_t *out, int max)
{
    nets_blob_t b;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    nets_load(&b);
    xSemaphoreGive(s_lock);
    int n = b.count < max ? b.count : max;
    memcpy(out, b.nets, n * sizeof(edge_wifi_net_t));
    return n;
}

esp_err_t edge_wifi_net_add(const char *ssid, const char *password)
{
    if (!ssid || !ssid[0] || strlen(ssid) > 32) return ESP_ERR_INVALID_ARG;
    if (password && strlen(password) > 64) return ESP_ERR_INVALID_ARG;
    if (password && password[0] && strlen(password) < 8) return ESP_ERR_INVALID_ARG;
    nets_blob_t b;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    nets_load(&b);
    int slot = -1;
    for (int i = 0; i < b.count; i++) {
        if (strcmp(b.nets[i].ssid, ssid) == 0) slot = i;
    }
    if (slot < 0) {
        if (b.count >= EDGE_MAX_WIFI_NETS) {
            /* Drop the oldest entry. */
            memmove(&b.nets[0], &b.nets[1], (EDGE_MAX_WIFI_NETS - 1) * sizeof(edge_wifi_net_t));
            b.count = EDGE_MAX_WIFI_NETS - 1;
        }
        slot = b.count++;
    }
    edge_strlcpy(b.nets[slot].ssid, ssid, sizeof(b.nets[slot].ssid));
    edge_strlcpy(b.nets[slot].password, password ? password : "", sizeof(b.nets[slot].password));
    esp_err_t err = save_blob("nets", &b, sizeof(b));
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t edge_wifi_net_remove(const char *ssid)
{
    nets_blob_t b;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    nets_load(&b);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (int i = 0; i < b.count; i++) {
        if (strcmp(b.nets[i].ssid, ssid) == 0) {
            memmove(&b.nets[i], &b.nets[i + 1], (b.count - i - 1) * sizeof(edge_wifi_net_t));
            b.count--;
            err = save_blob("nets", &b, sizeof(b));
            break;
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

uint32_t edge_seq_floor_get(void)
{
    uint32_t v = 0;
    load_u32("seqfloor", &v);
    return v;
}

esp_err_t edge_seq_floor_set(uint32_t floor)
{
    return save_u32("seqfloor", floor);
}

esp_err_t edge_config_factory_reset(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = nvs_flash_erase();
    xSemaphoreGive(s_lock);
    return err;
}
