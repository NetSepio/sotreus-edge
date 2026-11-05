#include "edge_config.h"

#include <string.h>

#include "bootloader_random.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "edge_meshmod.h"
#include "edge_util.h"
#include "mesh_crypto.h"

static const char *TAG = "config";
static const char *NS = "edge";

static SemaphoreHandle_t s_lock;
static edge_config_t s_cfg;
static edge_identity_t s_id;
static edge_mesh_mod_t s_mod;
static void mod_clamp(edge_mesh_mod_t *m);

static void random_text(char *out, size_t len, const char *alphabet)
{
    size_t n = strlen(alphabet);
    for (size_t i = 0; i < len; i++) out[i] = alphabet[esp_random() % n];
    out[len] = '\0';
}

/* Version 1 layout, kept to migrate older saved settings. */
typedef struct {
    char name[12];
    uint8_t psk[32];
    uint8_t psk_len;
} cfg_v1_channel_t;

typedef struct {
    uint16_t version;
    uint8_t mode, link_mode;
    bool ble_scan, ble_active, wifi_scan, wifi_probes, probe_ssid_plain;
    int8_t rssi_floor;
    uint16_t close_after_s, max_window_s, display_timeout_s;
    uint8_t region, preset, hop_limit, rebroadcast;
    int8_t tx_power;
    uint16_t channel_num;
    uint32_t node_num;
    char long_name[40];
    char short_name[5];
    uint32_t nodeinfo_interval_s;
    uint8_t n_channels;
    cfg_v1_channel_t channels[4];
} cfg_v1_t;

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
    c->channels[0].position_precision = 13; /* about 2.9 km, Meshtastic's default for public channels */
    edge_mesh_ext_t *x = &c->mx;
    x->role = EDGE_ROLE_CLIENT;
    x->pos_mode = EDGE_POS_OFF;
    x->pos_interval_s = 15 * 60;
    x->telemetry_interval_s = 30 * 60;
    x->neighbor_info = false;
    x->neighbor_interval_s = 6 * 3600;
    x->notify_mode = 1;
    x->notify_gpio = -1;
    x->detect_gpio = -1;
    x->detect_active_high = true;
    x->detect_min_interval_s = 45;
    snprintf(x->detect_name, sizeof(x->detect_name), "Sensor");
    x->pax_interval_s = 15 * 60;
    x->serial_rx = x->serial_tx = -1;
    x->serial_baud = 38400;
    static const char *const canned[] = {"OK", "On my way", "Need assistance", "All fine here"};
    x->n_canned = 4;
    for (int i = 0; i < 4; i++) snprintf(x->canned[i], sizeof(x->canned[i]), "%s", canned[i]);
    x->tx_enabled = true;
    x->rx_boosted = true;
    x->ignore_mqtt = true;
    x->ok_to_mqtt = false;
}

static void migrate_v1(const cfg_v1_t *o, edge_config_t *c)
{
    edge_config_defaults(c);
    c->mode = o->mode;
    c->link_mode = o->link_mode;
    c->ble_scan = o->ble_scan;
    c->ble_active = o->ble_active;
    c->wifi_scan = o->wifi_scan;
    c->wifi_probes = o->wifi_probes;
    c->probe_ssid_plain = o->probe_ssid_plain;
    c->rssi_floor = o->rssi_floor;
    c->close_after_s = o->close_after_s;
    c->max_window_s = o->max_window_s;
    c->display_timeout_s = o->display_timeout_s;
    c->region = o->region;
    c->preset = o->preset;
    c->hop_limit = o->hop_limit;
    c->rebroadcast = o->rebroadcast == 1 ? EDGE_REBROADCAST_LOCAL_ONLY : o->rebroadcast == 2 ? EDGE_REBROADCAST_NONE : EDGE_REBROADCAST_ALL;
    c->tx_power = o->tx_power;
    c->channel_num = o->channel_num;
    c->node_num = o->node_num;
    memcpy(c->long_name, o->long_name, sizeof(c->long_name));
    memcpy(c->short_name, o->short_name, sizeof(c->short_name));
    c->nodeinfo_interval_s = o->nodeinfo_interval_s;
    c->n_channels = o->n_channels > 4 ? 4 : o->n_channels;
    for (int i = 0; i < 4; i++) {
        memcpy(c->channels[i].name, o->channels[i].name, sizeof(c->channels[i].name));
        memcpy(c->channels[i].psk, o->channels[i].psk, sizeof(c->channels[i].psk));
        c->channels[i].psk_len = o->channels[i].psk_len;
        bool public_key = o->channels[i].psk_len == 1 && o->channels[i].psk[0] == 1;
        c->channels[i].position_precision = public_key ? 13 : 32;
    }
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
    edge_mesh_ext_t *x = &c->mx;
    if (x->role > EDGE_ROLE_ROUTER_LATE || x->role == 3) x->role = EDGE_ROLE_CLIENT;
    if (x->pos_mode > EDGE_POS_CLIENT) x->pos_mode = EDGE_POS_OFF;
    if (x->pos_interval_s < 60) x->pos_interval_s = 60;
    if (x->telemetry_interval_s && x->telemetry_interval_s < 300) x->telemetry_interval_s = 300;
    if (x->neighbor_interval_s < 3600) x->neighbor_interval_s = 3600;
    if (x->range_test_interval_s && x->range_test_interval_s < 15) x->range_test_interval_s = 15;
    if (x->notify_mode > 2) x->notify_mode = 1;
    if (x->detect_min_interval_s < 5) x->detect_min_interval_s = 5;
    if (x->pax_interval_s < 60) x->pax_interval_s = 60;
    if (x->serial_baud < 1200 || x->serial_baud > 921600) x->serial_baud = 38400;
    if (x->n_canned > EDGE_CANNED_MAX) x->n_canned = EDGE_CANNED_MAX;
    for (int i = 0; i < EDGE_CANNED_MAX; i++) x->canned[i][sizeof(x->canned[i]) - 1] = '\0';
    x->detect_name[sizeof(x->detect_name) - 1] = '\0';
    if (x->n_ignore > EDGE_IGNORE_MAX) x->n_ignore = EDGE_IGNORE_MAX;
    if (x->freq_offset_hz > 1000000 || x->freq_offset_hz < -1000000) x->freq_offset_hz = 0;
    if (c->n_channels < 1) c->n_channels = 1;
    if (c->n_channels > EDGE_MESH_MAX_CHANNELS) c->n_channels = EDGE_MESH_MAX_CHANNELS;
    for (int i = 0; i < EDGE_MESH_MAX_CHANNELS; i++) {
        edge_mesh_channel_t *ch = &c->channels[i];
        ch->name[sizeof(ch->name) - 1] = '\0';
        if (ch->psk_len != 0 && ch->psk_len != 1 && ch->psk_len != 16 && ch->psk_len != 32) {
            ch->psk_len = 1;
            ch->psk[0] = 1;
        }
        if (ch->position_precision > 32) ch->position_precision = 32;
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

    cfg_v1_t old;
    if (load_blob("cfg", &s_cfg, sizeof(s_cfg)) == ESP_OK && s_cfg.version == EDGE_CONFIG_VERSION) {
        /* current */
    } else if (load_blob("cfg", &old, sizeof(old)) == ESP_OK && old.version == 1) {
        ESP_LOGI(TAG, "migrating settings from version 1");
        migrate_v1(&old, &s_cfg);
        save_blob("cfg", &s_cfg, sizeof(s_cfg));
    } else {
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
    edge_mod_defaults(&s_mod);
    esp_err_t me = load_blob("meshmod", &s_mod, sizeof(s_mod));
    if (me == ESP_ERR_INVALID_SIZE && s_mod.version == 1) {
        /* version 1 was shorter: fields added since keep their defaults */
        s_mod.tak_team = s_mod.tak_role = 0;
        s_mod.version = EDGE_MOD_VERSION;
        save_blob("meshmod", &s_mod, sizeof(s_mod));
    } else if (me != ESP_OK || s_mod.version != EDGE_MOD_VERSION) {
        edge_mod_defaults(&s_mod);
        save_blob("meshmod", &s_mod, sizeof(s_mod));
    }
    mod_clamp(&s_mod);
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

size_t edge_ctx_get(char *out, size_t cap)
{
    nvs_handle_t h;
    if (cap == 0 || nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return 0;
    size_t len = cap;
    esp_err_t err = nvs_get_str(h, "ctx", out, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        out[0] = '\0';
        return 0;
    }
    return len ? len - 1 : 0;
}

esp_err_t edge_ctx_set(const char *json)
{
    if (!json || strlen(json) >= EDGE_CTX_MAX) return ESP_ERR_INVALID_SIZE;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, "ctx", json);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

bool edge_tz_get(int32_t *minutes)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    esp_err_t err = nvs_get_i32(h, "tzoff", minutes);
    nvs_close(h);
    return err == ESP_OK;
}

esp_err_t edge_tz_set(int32_t minutes)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_i32(h, "tzoff", minutes);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t edge_pki_get(uint8_t priv[32], uint8_t pub[32])
{
    uint8_t kp[64];
    if (load_blob("pki", kp, sizeof(kp)) != ESP_OK) {
        esp_err_t err = edge_pki_regenerate();
        if (err != ESP_OK) return err;
        if (load_blob("pki", kp, sizeof(kp)) != ESP_OK) return ESP_FAIL;
    }
    memcpy(priv, kp, 32);
    memcpy(pub, kp + 32, 32);
    memset(kp, 0, sizeof(kp));
    return ESP_OK;
}

esp_err_t edge_pki_regenerate(void)
{
    uint8_t kp[64];
    esp_fill_random(kp, 32);
    mesh_x25519_clamp(kp);
    mesh_x25519_public(kp + 32, kp);
    esp_err_t err = save_blob("pki", kp, sizeof(kp));
    memset(kp, 0, sizeof(kp));
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

/* ---- Mesh module settings ("meshmod") ------------------------------------------------ */

void edge_mod_defaults(edge_mesh_mod_t *m)
{
    memset(m, 0, sizeof(*m));
    m->version = EDGE_MOD_VERSION;
    m->admin_enabled = true;          /* open by default, as the owner chose; the page warns */
    m->admin_mode = EDGE_ADMIN_OPEN;
    m->admin_channel = false;
    m->sf_enabled = false;
    m->sf_heartbeat = true;
    m->sf_return_max = 25;
    m->sf_return_window_min = 240;
    edge_strlcpy(m->mqtt_address, "mqtt.meshtastic.org", sizeof(m->mqtt_address));
    edge_strlcpy(m->mqtt_username, "meshdev", sizeof(m->mqtt_username));
    edge_strlcpy(m->mqtt_password, "large4cats", sizeof(m->mqtt_password));
    m->mqtt_encryption = true;
    m->mqtt_map_interval_s = 3600;
    m->mqtt_map_precision = 14;
    m->audio_ptt = -1;
    m->i2s_ws = m->i2s_sd = m->i2s_din = m->i2s_sck = -1;
}

/* GPIOs the Heltec V3 uses itself: button, battery ADC, LoRa SPI, OLED, USB, flash, LED, Vext, console UART. */
bool edge_pin_protected(int g)
{
    static const uint8_t P[] = {0, 1, 8, 9, 10, 11, 12, 13, 14, 17, 18, 19, 20, 21, 26, 27, 28, 29, 30, 31, 32, 35, 36, 37, 43, 44};
    if (g < 0 || g > 48) return true;
    for (size_t i = 0; i < sizeof(P); i++) {
        if (P[i] == g) return true;
    }
    return false;
}

static void mod_clamp(edge_mesh_mod_t *m)
{
    m->version = EDGE_MOD_VERSION;
    if (m->admin_mode > EDGE_ADMIN_KEYS) m->admin_mode = EDGE_ADMIN_OPEN;
    if (m->n_admin_keys > EDGE_ADMIN_KEYS_MAX) m->n_admin_keys = EDGE_ADMIN_KEYS_MAX;
    if (m->n_pins > EDGE_RHW_PINS_MAX) m->n_pins = EDGE_RHW_PINS_MAX;
    int k = 0;
    for (int i = 0; i < m->n_pins; i++) {
        if (edge_pin_protected(m->pins[i].gpio)) continue;
        if (m->pins[i].type != EDGE_RHW_READ && m->pins[i].type != EDGE_RHW_WRITE) m->pins[i].type = EDGE_RHW_READ;
        m->pins[i].name[sizeof(m->pins[i].name) - 1] = 0;
        m->pins[k++] = m->pins[i];
    }
    m->n_pins = (uint8_t)k;
    if (m->sf_return_max == 0 || m->sf_return_max > 100) m->sf_return_max = 25;
    if (m->sf_return_window_min == 0 || m->sf_return_window_min > 7 * 24 * 60) m->sf_return_window_min = 240;
    m->mqtt_address[sizeof(m->mqtt_address) - 1] = 0;
    m->mqtt_username[sizeof(m->mqtt_username) - 1] = 0;
    m->mqtt_password[sizeof(m->mqtt_password) - 1] = 0;
    m->mqtt_root[sizeof(m->mqtt_root) - 1] = 0;
    if (!m->mqtt_address[0]) edge_strlcpy(m->mqtt_address, "mqtt.meshtastic.org", sizeof(m->mqtt_address));
    if (m->mqtt_map_interval_s < 900) m->mqtt_map_interval_s = 900;
    if (m->mqtt_map_precision > 32) m->mqtt_map_precision = 14;
    if (m->audio_bitrate > 10) m->audio_bitrate = 0;
    if (m->tak_team > 14) m->tak_team = 0;
    if (m->tak_role > 8) m->tak_role = 0;
}

void edge_mod_get(edge_mesh_mod_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_mod;
    xSemaphoreGive(s_lock);
}

esp_err_t edge_mod_set(const edge_mesh_mod_t *in)
{
    edge_mesh_mod_t m = *in;
    mod_clamp(&m);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_mod = m;
    esp_err_t err = save_blob("meshmod", &s_mod, sizeof(s_mod));
    xSemaphoreGive(s_lock);
    return err;
}
