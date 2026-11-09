#include "edge_ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "edge_api.h"
#include "edge_config.h"
#include "edge_log.h"
#include "edge_stream.h"
#include "edge_time.h"
#include "mesh.h"
#include "sense.h"

static const char *TAG = "ble";

void ble_store_config_init(void);

#define MAX_CONN        CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define RX_BUF_SIZE     1536
#define FOREVER         INT64_MAX

/* 5e0e000X-5307-4e75-8d9a-ed9e5e0e0001, little-endian byte order. */
#define EDGE_UUID(x) BLE_UUID128_INIT(0x01, 0x00, 0x0e, 0x5e, 0x9e, 0xed, 0x9a, 0x8d, 0x75, 0x4e, 0x07, 0x53, \
                                      (x), 0x00, 0x0e, 0x5e)
static const ble_uuid128_t UUID_SVC = EDGE_UUID(0x01);
static const ble_uuid128_t UUID_INFO = EDGE_UUID(0x02);
static const ble_uuid128_t UUID_CTRL = EDGE_UUID(0x03);
static const ble_uuid128_t UUID_EVENTS = EDGE_UUID(0x04);
static const ble_uuid128_t UUID_RECORDS = EDGE_UUID(0x05);

typedef struct {
    bool used;
    uint16_t handle;
    uint16_t mtu;
    bool secured;
    bool sub_events;
    bool sub_records;
    bool dle_set;      /* the settled payload step was scheduled once on this link */
    bool full_size;    /* the settled step keeps 251-octet packets (the phone handles them) */
    bool settled;      /* the settled step has run */
    bool limited;      /* the settled step limited this link's payload */
    uint16_t gen;      /* tells this link apart from a later one on the same handle */
    size_t rx_len;
    char *rx;
} conn_t;

static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_rec_lock; /* one record at a time on the records characteristic (sync and live) */
static conn_t s_conn[MAX_CONN];
static uint16_t h_info, h_ctrl, h_events, h_records;
static uint8_t s_own_addr_type;
static volatile bool s_synced;
static volatile bool s_closing; /* a planned restart is under way: no more advertising */
static volatile bool s_scan_want, s_scan_active_mode, s_scanning;
static volatile int64_t s_pair_until;
/* Fast advertising after boot and after a link drops, so a bonded phone's background
 * reconnect finds the Edge in seconds; then back to once a second. */
#define FAST_ADV_MS 30000
static volatile int64_t s_fast_until;
/* Diagnostic: cap on the link-layer payload the Edge sends (0 = controller default). Set from the
 * USB console only, to tell a phone's trouble with long packets apart from other link faults. */
static volatile uint16_t s_tx_octets_cap;
/* Link-layer payload new connections start with. The controller's own default (251) makes the Edge
 * open a data-length procedure the instant a phone connects; on an Android 9 Huawei that collided
 * with the phone's own procedures. 27 opens none. */
#define BOOT_TX_OCTETS 27
/* Applied this long after a paired phone's link is encrypted, when its own setup is done. */
#define SETTLED_DLE_MS 3000
/* Bonded phones whose app said they handle 251-octet packets (`ble_link`): their links skip the
 * limit. Kept in NVS by identity address; a factory reset erases it with the bonds. */
#define FULL_MAX CONFIG_BT_NIMBLE_MAX_BONDS
static ble_addr_t s_full[FULL_MAX];
static int s_full_n;
static uint16_t s_gen;
static esp_err_t full_save(const ble_addr_t *list, int n);
static volatile bool s_passkey_active;
static volatile uint32_t s_passkey;
static bool s_adv_pairing_mode;
static char s_name[24];
static esp_timer_handle_t s_tick;

/* True if this phone has a stored bond (the link can re-encrypt without pairing). */
static bool peer_bonded(const ble_addr_t *addr)
{
    struct ble_store_key_sec key;
    struct ble_store_value_sec value;
    memset(&key, 0, sizeof(key));
    key.peer_addr = *addr;
    return ble_store_read_peer_sec(&key, &value) == 0;
}

/* The phone's address for the log, shortened the way Android logs it (..:d7:52). */
static void peer_tag(const struct ble_gap_conn_desc *d, char *out, size_t n)
{
    const uint8_t *v = d->peer_id_addr.val;
    bool via_rpa = memcmp(d->peer_ota_addr.val, v, 6) != 0;
    snprintf(out, n, "..:%02x:%02x %s%s%s", v[1], v[0], d->peer_id_addr.type == BLE_ADDR_PUBLIC ? "public" : "random",
             via_rpa ? ", via private address" : "", peer_bonded(&d->peer_id_addr) ? ", paired" : "");
}

/* ---- Links NimBLE accepted but never reported ------------------------------------
 * As a peripheral, NimBLE reports a link (BLE_GAP_EVENT_CONNECT) only after reading the
 * phone's version and features. If that read is refused, the link is never reported, yet
 * it holds a connection slot, and advertising fails with BLE_HS_ENOMEM while it lasts.
 * A check on the host task ends such a link after UNREPORTED_MS. */
#define UNREPORTED_MS 10000

typedef struct {
    uint16_t handle;
    bool used, ended;
    int64_t since;
} unreported_t;

static conn_t *conn_find(uint16_t h);
/* Defined in NimBLE's ble_gap.c (ESP-IDF 6.0.1) but missing from its public header. */
void ble_gap_conn_foreach_handle(ble_gap_conn_foreach_handle_fn *cb, void *arg);

static unreported_t s_unrep[MAX_CONN + 1];
static struct ble_npl_callout s_reap_co;
static bool s_reap_init;

typedef struct {
    int n;
    uint16_t h[MAX_CONN + 2];
} handles_t;

static int collect_handle(uint16_t h, void *arg)
{
    handles_t *l = arg;
    if (l->n < (int)(sizeof(l->h) / sizeof(l->h[0]))) l->h[l->n++] = h;
    return 0;
}

static void reap_unreported(void)
{
    handles_t l = {0};
    ble_gap_conn_foreach_handle(collect_handle, &l); /* on the host task: the list cannot change under us */
    int64_t now = edge_uptime_ms();
    uint16_t end[MAX_CONN + 2];
    int n_end = 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_CONN + 1; i++) { /* forget links that are gone */
        bool alive = false;
        for (int j = 0; j < l.n; j++) alive |= s_unrep[i].used && s_unrep[i].handle == l.h[j];
        if (!alive) s_unrep[i].used = false;
    }
    for (int j = 0; j < l.n; j++) {
        if (conn_find(l.h[j])) continue; /* reported: the app knows it */
        unreported_t *u = NULL, *free_slot = NULL;
        for (int i = 0; i < MAX_CONN + 1; i++) {
            if (s_unrep[i].used && s_unrep[i].handle == l.h[j]) u = &s_unrep[i];
            else if (!s_unrep[i].used && !free_slot) free_slot = &s_unrep[i];
        }
        if (!u && free_slot) {
            *free_slot = (unreported_t){.handle = l.h[j], .used = true, .since = now};
            continue;
        }
        if (u && !u->ended && now - u->since >= UNREPORTED_MS) {
            u->ended = true;
            end[n_end++] = u->handle;
        }
    }
    xSemaphoreGive(s_lock);
    for (int k = 0; k < n_end; k++) {
        struct ble_gap_conn_desc d;
        char tag[48] = "unknown";
        if (ble_gap_conn_find(end[k], &d) == 0) peer_tag(&d, tag, sizeof(tag));
        int rc = ble_gap_terminate(end[k], BLE_ERR_REM_USER_CONN_TERM);
        ESP_LOGW(TAG, "conn %u never reported by the Bluetooth stack after %d s: ending it (peer %s, rc %d)", end[k],
                 UNREPORTED_MS / 1000, tag, rc);
    }
}

static int conn_count(void);

/* Runs on the host task, which may walk the link list. */
static void close_cb(struct ble_npl_event *ev)
{
    if (ble_gap_adv_active()) ble_gap_adv_stop();
    handles_t l = {0};
    ble_gap_conn_foreach_handle(collect_handle, &l);
    for (int k = 0; k < l.n; k++) ble_gap_terminate(l.h[k], BLE_ERR_REM_USER_CONN_TERM);
    ESP_LOGI(TAG, "restarting: ending %d link(s)", l.n);
}

void edge_ble_close(uint32_t wait_ms)
{
    s_closing = true;
    if (!s_synced) return;
    static struct ble_npl_event ev;
    ble_npl_event_init(&ev, close_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &ev);
    for (uint32_t t = 0; t < wait_ms; t += 20) {
        vTaskDelay(pdMS_TO_TICKS(20));
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int n = conn_count();
        xSemaphoreGive(s_lock);
        if (!n) break;
    }
}

static void reap_cb(struct ble_npl_event *ev)
{
    reap_unreported();
    ble_npl_callout_reset(&s_reap_co, ble_npl_time_ms_to_ticks32(1000));
}

static int gap_event(struct ble_gap_event *event, void *arg);
static void advertise(void);

/* ---- Connection table ------------------------------------------------------- */

static conn_t *conn_find(uint16_t h)
{
    for (int i = 0; i < MAX_CONN; i++) {
        if (s_conn[i].used && s_conn[i].handle == h) return &s_conn[i];
    }
    return NULL;
}

static conn_t *conn_add(uint16_t h)
{
    for (int i = 0; i < MAX_CONN; i++) {
        if (!s_conn[i].used) {
            char *rx = s_conn[i].rx ? s_conn[i].rx : malloc(RX_BUF_SIZE);
            memset(&s_conn[i], 0, sizeof(s_conn[i]));
            s_conn[i].used = true;
            s_conn[i].handle = h;
            s_conn[i].mtu = 23;
            s_conn[i].gen = ++s_gen;
            s_conn[i].rx = rx;
            return &s_conn[i];
        }
    }
    return NULL;
}

static int conn_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_CONN; i++) n += s_conn[i].used;
    return n;
}

static int bond_count(void)
{
    int n = 0;
    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &n) != 0) return 0;
    return n;
}

static bool link_secured(uint16_t h)
{
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(h, &d) != 0) return false;
    return d.sec_state.encrypted && d.sec_state.authenticated;
}

static bool pairing_open(void)
{
    return s_pair_until > edge_uptime_ms();
}

/* ---- Notifications ---------------------------------------------------------- */

static int notify_chunk(uint16_t conn, uint16_t attr, const uint8_t *data, size_t len)
{
    for (int attempt = 0; attempt < 200; attempt++) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
        if (!om) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int rc = ble_gatts_notify_custom(conn, attr, om); /* consumes om */
        if (rc == 0) return 0;
        if (rc != BLE_HS_ENOMEM && rc != BLE_HS_EBUSY) return rc;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return BLE_HS_ETIMEOUT;
}

static int notify_stream(uint16_t conn, uint16_t attr, uint16_t mtu, const uint8_t *data, size_t len)
{
    size_t chunk = mtu > 3 ? mtu - 3 : 20;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off < chunk ? len - off : chunk;
        int rc = notify_chunk(conn, attr, data + off, n);
        if (rc) return rc;
    }
    return 0;
}

void edge_ble_send_line(int conn, const char *line, size_t len)
{
    conn_t targets[MAX_CONN];
    int n = 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_CONN; i++) {
        conn_t *c = &s_conn[i];
        if (!c->used || !c->secured || !c->sub_events) continue;
        if (conn >= 0 && c->handle != conn) continue;
        targets[n++] = *c;
    }
    xSemaphoreGive(s_lock);
    for (int i = 0; i < n; i++) {
        notify_stream(targets[i].handle, h_events, targets[i].mtu, (const uint8_t *)line, len);
    }
}

bool edge_ble_has_subscriber(void)
{
    for (int i = 0; i < MAX_CONN; i++) {
        if (s_conn[i].used && s_conn[i].secured && s_conn[i].sub_events) return true;
    }
    return false;
}

int edge_ble_notify_record(int conn, const edge_rec_t *rec)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    conn_t *c = conn_find((uint16_t)conn);
    bool ok = c && c->secured && c->sub_records;
    uint16_t mtu = c ? c->mtu : 23;
    xSemaphoreGive(s_lock);
    if (!ok) return -1;
    xSemaphoreTake(s_rec_lock, portMAX_DELAY);
    int rc = notify_stream((uint16_t)conn, h_records, mtu, (const uint8_t *)rec, sizeof(edge_rec_t));
    xSemaphoreGive(s_rec_lock);
    return rc ? -1 : 0;
}

bool edge_ble_records_ready(int conn)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    conn_t *c = conn_find((uint16_t)conn);
    bool ok = c && c->secured && c->sub_records;
    xSemaphoreGive(s_lock);
    return ok;
}

int edge_ble_stream_records(int conn, uint32_t from, uint32_t limit, uint32_t *next)
{
    const int batch = 8;
    edge_rec_t *recs = malloc(batch * sizeof(edge_rec_t));
    if (!recs) return -1;
    uint32_t sent = 0;
    uint32_t cursor = from;
    int result = 0;
    while (sent < limit) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find((uint16_t)conn);
        bool ok = c && c->secured && c->sub_records;
        uint16_t mtu = c ? c->mtu : 23;
        xSemaphoreGive(s_lock);
        if (!ok) {
            result = -1;
            break;
        }
        int want = (int)((limit - sent) < (uint32_t)batch ? (limit - sent) : (uint32_t)batch);
        int n = edge_log_read_from(cursor, recs, want);
        if (n <= 0) break;
        for (int i = 0; i < n; i++) {
            xSemaphoreTake(s_rec_lock, portMAX_DELAY);
            int rc = notify_stream((uint16_t)conn, h_records, mtu, (const uint8_t *)&recs[i], sizeof(edge_rec_t));
            xSemaphoreGive(s_rec_lock);
            if (rc) {
                result = -1;
                break;
            }
            sent++;
            cursor = recs[i].seq + 1;
        }
        if (result < 0) break;
    }
    free(recs);
    *next = cursor;
    return result < 0 ? -1 : (int)sent;
}

/* ---- GATT ------------------------------------------------------------------- */

static int chr_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (!link_secured(conn)) return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;

    if (attr == h_info && ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        cJSON *info = edge_api_info_json();
        char *s = cJSON_PrintUnformatted(info);
        cJSON_Delete(info);
        if (!s) return BLE_ATT_ERR_INSUFFICIENT_RES;
        int rc = os_mbuf_append(ctxt->om, s, strlen(s));
        free(s);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (attr == h_ctrl && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find(conn);
        if (!c || !c->rx) {
            xSemaphoreGive(s_lock);
            return BLE_ATT_ERR_UNLIKELY;
        }
        if (c->rx_len + len >= RX_BUF_SIZE) {
            c->rx_len = 0; /* oversized command: drop it */
            xSemaphoreGive(s_lock);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t got = 0;
        ble_hs_mbuf_to_flat(ctxt->om, c->rx + c->rx_len, len, &got);
        c->rx_len += got;
        /* Hand every complete line to the API worker. */
        char *nl;
        while ((nl = memchr(c->rx, '\n', c->rx_len)) != NULL) {
            size_t line_len = (size_t)(nl - c->rx);
            char *line = malloc(line_len + 1);
            if (line) {
                memcpy(line, c->rx, line_len);
                line[line_len] = '\0';
                edge_api_submit(EDGE_SINK_BLE, conn, line);
            }
            c->rx_len -= line_len + 1;
            memmove(c->rx, nl + 1, c->rx_len);
        }
        xSemaphoreGive(s_lock);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &UUID_INFO.u,
                .access_cb = chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN,
                .val_handle = &h_info,
            },
            {
                .uuid = &UUID_CTRL.u,
                .access_cb = chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC |
                         BLE_GATT_CHR_F_WRITE_AUTHEN,
                .val_handle = &h_ctrl,
            },
            {
                .uuid = &UUID_EVENTS.u,
                .access_cb = chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_events,
            },
            {
                .uuid = &UUID_RECORDS.u,
                .access_cb = chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_records,
            },
            {0},
        },
    },
    {0},
};

/* ---- Scanning --------------------------------------------------------------- */

static void scan_apply(void)
{
    if (!s_synced) return;
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
    if (!s_scan_want) return;
    struct ble_gap_disc_params dp = {
        .itvl = 160,   /* 100 ms */
        .window = 80,  /* 50 ms: leaves air time for Wi-Fi and the app link */
        .filter_policy = 0,
        .limited = 0,
        .passive = s_scan_active_mode ? 0 : 1,
        .filter_duplicates = 0,
    };
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &dp, gap_event, NULL);
    if (rc == 0) {
        s_scanning = true;
    } else {
        ESP_LOGW(TAG, "scan start failed: %d", rc);
    }
}

void edge_ble_scan_start(bool active)
{
    s_scan_want = true;
    s_scan_active_mode = active;
    scan_apply();
}

void edge_ble_scan_stop(void)
{
    s_scan_want = false;
    scan_apply();
}

static void on_disc(const struct ble_gap_disc_desc *d)
{
    sense_ble_report_t r = {
        .addr_type = d->addr.type & 1,
        .evt_type = d->event_type,
        .rssi = d->rssi,
    };
    for (int i = 0; i < 6; i++) r.addr[i] = d->addr.val[5 - i];
    r.len = d->length_data > 31 ? 31 : d->length_data;
    memcpy(r.data, d->data, r.len);
    sense_feed_ble(&r);
    mesh_pax_seen(false, r.addr); /* paxcounter (Mesh mode, opt-in): counts only */
}

/* ---- Advertising / pairing ---------------------------------------------------- */

static void advertise(void)
{
    if (!s_synced || s_closing) return;
    if (ble_gap_adv_active()) ble_gap_adv_stop();
    if (conn_count() >= MAX_CONN) return;

    bool pairing = pairing_open();
    s_adv_pairing_mode = pairing;
    struct ble_hs_adv_fields f = {0};
    f.flags = (pairing ? BLE_HS_ADV_F_DISC_GEN : 0) | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128 = (ble_uuid128_t *)&UUID_SVC;
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc) ESP_LOGW(TAG, "adv fields: %d", rc);

    /* The name (with the random suffix shown on the display) only while pairing. */
    struct ble_hs_adv_fields rsp = {0};
    if (pairing) {
        rsp.name = (const uint8_t *)s_name;
        rsp.name_len = (uint8_t)strlen(s_name);
        rsp.name_is_complete = 1;
    }
    ble_gap_adv_rsp_set_fields(&rsp);

    int64_t now = edge_uptime_ms();
    bool fast = pairing || now < s_fast_until;
    /* A fast window ends by itself (ADV_COMPLETE), which restarts advertising slowly. */
    int32_t duration = pairing || !fast ? BLE_HS_FOREVER : (int32_t)(s_fast_until - now);
    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = pairing ? BLE_GAP_DISC_MODE_GEN : BLE_GAP_DISC_MODE_NON,
        .itvl_min = fast ? 160 : 1600,  /* 100 ms while pairing or reconnecting, 1 s otherwise */
        .itvl_max = fast ? 240 : 1920,
    };
    rc = ble_gap_adv_start(s_own_addr_type, NULL, duration, &p, gap_event, NULL);
    if (rc == BLE_HS_ENOMEM) {
        ESP_LOGW(TAG, "adv start: no free link slot (rc %d)", rc);
    } else if (rc && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "adv start: %d", rc);
    }
}

void edge_ble_open_pairing(uint32_t seconds)
{
    if (seconds == 0) seconds = 120;
    s_pair_until = edge_uptime_ms() + (int64_t)seconds * 1000;
    ESP_LOGI(TAG, "pairing open for %lu s", (unsigned long)seconds);
    advertise();
}

static void close_pairing(void)
{
    s_pair_until = 0;
    s_passkey_active = false;
    advertise();
}

esp_err_t edge_ble_forget_all(void)
{
    int rc = ble_store_clear();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_full_n = 0;
    xSemaphoreGive(s_lock);
    full_save(NULL, 0);
    s_pair_until = FOREVER;
    advertise();
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

static void tick_cb(void *arg)
{
    /* Close an expired pairing window and refresh advertising. */
    if (s_adv_pairing_mode && !pairing_open()) {
        s_passkey_active = false;
        advertise();
    }
}

/* ---- GAP events --------------------------------------------------------------- */

/* Suggested default for new connections: changes no live link and starts no procedure. */
static void write_default_data_len(uint16_t octets)
{
    uint16_t o = octets ? octets : 251;
    int rc = ble_gap_write_sugg_def_data_len(o, (uint16_t)((o + 14) * 8)); /* 1M PHY airtime */
    ESP_LOGI(TAG, "new links start with %u-octet link payload (rc %d)", o, rc);
}

static void log_conn_params(uint16_t conn, const char *what, int status)
{
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn, &d) != 0) return;
    ESP_LOGI(TAG, "conn %u %s: interval %u.%02u ms, latency %u, timeout %u ms (status %d)", conn, what,
             d.conn_itvl * 5 / 4, (d.conn_itvl * 125) % 100, d.conn_latency, d.supervision_timeout * 10, status);
}

/* Limits a settled link's payload: a separate step, so it never collides with the phone's
 * connection-parameter or encryption procedures at connect. A phone that handles full-size
 * packets keeps 251. `arg` packs the link's generation (high 16 bits) and handle. */
static void settled_dle_task(void *arg)
{
    uint32_t v = (uint32_t)(uintptr_t)arg;
    uint16_t h = (uint16_t)(v & 0xffff), gen = (uint16_t)(v >> 16);
    vTaskDelay(pdMS_TO_TICKS(SETTLED_DLE_MS));
    uint16_t o = s_tx_octets_cap;
    bool mine = false, full = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    conn_t *c = conn_find(h);
    if (c && c->gen == gen) {
        mine = true;
        full = c->full_size;
        c->settled = true;
        c->limited = o && !full;
    }
    xSemaphoreGive(s_lock);
    if (mine && o && full) {
        ESP_LOGI(TAG, "conn %u settled: full-size link payload kept (the phone handles it)", h);
    } else if (mine && o) {
        int rc = ble_gap_set_data_len(h, o, (uint16_t)((o + 14) * 8));
        ESP_LOGI(TAG, "conn %u settled: link payload limited to %u octets (rc %d)", h, o, rc);
        if (rc != 0) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            c = conn_find(h);
            if (c && c->gen == gen) c->limited = false;
            xSemaphoreGive(s_lock);
        }
    }
    vTaskDelete(NULL);
}

void edge_ble_set_tx_octets(uint16_t octets)
{
    /* New links only: a request on a live link can collide with one still pending. */
    s_tx_octets_cap = octets;
    write_default_data_len(octets);
}

/* ---- Full-size packets, per phone ------------------------------------------------- */

#define FULL_NS  "edge_ble"
#define FULL_KEY "full"

static void full_load(void)
{
    nvs_handle_t nh;
    size_t len = sizeof(s_full);
    s_full_n = 0;
    if (nvs_open(FULL_NS, NVS_READONLY, &nh) != ESP_OK) return;
    if (nvs_get_blob(nh, FULL_KEY, s_full, &len) == ESP_OK) s_full_n = (int)(len / sizeof(s_full[0]));
    nvs_close(nh);
}

static esp_err_t full_save(const ble_addr_t *list, int n)
{
    nvs_handle_t nh;
    esp_err_t err = nvs_open(FULL_NS, NVS_READWRITE, &nh);
    if (err != ESP_OK) return err;
    err = n ? nvs_set_blob(nh, FULL_KEY, list, (size_t)n * sizeof(list[0])) : nvs_erase_key(nh, FULL_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    if (err == ESP_OK) err = nvs_commit(nh);
    nvs_close(nh);
    return err;
}

/* Caller holds s_lock. */
static int full_index(const ble_addr_t *a)
{
    for (int i = 0; i < s_full_n; i++) {
        if (s_full[i].type == a->type && memcmp(s_full[i].val, a->val, 6) == 0) return i;
    }
    return -1;
}

/* Caller holds s_lock. Copies the list out so it can be saved after the lock is released. */
static int full_remove_locked(const ble_addr_t *a, ble_addr_t *copy)
{
    int i = full_index(a);
    if (i < 0) return -1;
    memmove(&s_full[i], &s_full[i + 1], (size_t)(s_full_n - i - 1) * sizeof(s_full[0]));
    s_full_n--;
    memcpy(copy, s_full, (size_t)s_full_n * sizeof(s_full[0]));
    return s_full_n;
}

/* A phone whose bond is gone starts with the limit again until its app asks. */
static void full_forget(const ble_addr_t *a)
{
    ble_addr_t copy[FULL_MAX];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = full_remove_locked(a, copy);
    xSemaphoreGive(s_lock);
    if (n >= 0) full_save(copy, n);
}

esp_err_t edge_ble_full_size(int conn, int set, bool *stored, bool *this_link)
{
    struct ble_gap_conn_desc d;
    if (conn < 0 || conn > 0xffff || ble_gap_conn_find((uint16_t)conn, &d) != 0) return ESP_ERR_NOT_FOUND;
    if (!peer_bonded(&d.peer_id_addr)) return ESP_ERR_INVALID_STATE;
    ble_addr_t copy[FULL_MAX];
    int n = -1;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    conn_t *c = conn_find((uint16_t)conn);
    if (!c) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_NOT_FOUND;
    }
    int i = full_index(&d.peer_id_addr);
    if (set == 1 && i < 0) {
        if (s_full_n == FULL_MAX) { /* the oldest entry goes: its phone may no longer be bonded */
            memmove(&s_full[0], &s_full[1], (size_t)(FULL_MAX - 1) * sizeof(s_full[0]));
            s_full_n--;
        }
        s_full[s_full_n++] = d.peer_id_addr;
        n = s_full_n;
        memcpy(copy, s_full, (size_t)n * sizeof(s_full[0]));
    } else if (set == 0 && i >= 0) {
        n = full_remove_locked(&d.peer_id_addr, copy);
    }
    if (set >= 0 && !c->settled) c->full_size = set == 1; /* this link follows while its step is pending */
    *stored = full_index(&d.peer_id_addr) >= 0;
    *this_link = c->settled ? !c->limited : !(c->dle_set && s_tx_octets_cap && !c->full_size);
    xSemaphoreGive(s_lock);
    if (n >= 0) {
        ESP_LOGI(TAG, "conn %d: full-size link packets %s for this phone (this link %s)", conn,
                 *stored ? "on" : "off", *this_link ? "full size" : "limited");
        if (full_save(copy, n) != ESP_OK) return ESP_FAIL;
    }
    return ESP_OK;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        on_disc(&event->disc);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        if (s_scan_want) scan_apply();
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            conn_t *c = conn_add(event->connect.conn_handle);
            xSemaphoreGive(s_lock);
            struct ble_gap_conn_desc pd;
            char tag[48] = "unknown";
            if (ble_gap_conn_find(event->connect.conn_handle, &pd) == 0) peer_tag(&pd, tag, sizeof(tag));
            if (!c) {
                ESP_LOGW(TAG, "conn %u refused: no free link slot (peer %s)", event->connect.conn_handle, tag);
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            } else {
                ESP_LOGI(TAG, "connected (%d): peer %s", event->connect.conn_handle, tag);
                log_conn_params(event->connect.conn_handle, "opened", 0);
                /* Bonded phones re-encrypt right away. A new phone starts pairing itself: if the Edge
                 * asked first, Android would ask its owner for consent before pairing, and the
                 * passkey would time out waiting behind a notification. */
                struct ble_gap_conn_desc d;
                if (ble_gap_conn_find(event->connect.conn_handle, &d) == 0 && peer_bonded(&d.peer_id_addr)) {
                    ble_gap_security_initiate(event->connect.conn_handle);
                }
            }
        } else {
            /* NimBLE also reports a link that ended before it was reported this way (BLE_HS_EAGAIN). */
            ESP_LOGI(TAG, "conn %u closed before it opened (status %d)", event->connect.conn_handle,
                     event->connect.status);
        }
        advertise();
        if (s_scan_want && !s_scanning) scan_apply();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find(event->disconnect.conn.conn_handle);
        if (c) c->used = false;
        xSemaphoreGive(s_lock);
        edge_stream_link_down(event->disconnect.conn.conn_handle); /* unconfirmed records go to flash */
        s_passkey_active = false;
        char tag[48];
        peer_tag(&event->disconnect.conn, tag, sizeof(tag));
        ESP_LOGI(TAG, "disconnected (reason 0x%x): conn %u, peer %s", event->disconnect.reason,
                 event->disconnect.conn.conn_handle, tag);
        s_fast_until = edge_uptime_ms() + FAST_ADV_MS;
        advertise();
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
        log_conn_params(event->conn_update.conn_handle, "updated", event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        ESP_LOGI(TAG, "conn %u phy: tx %u rx %u (status %d)", event->phy_updated.conn_handle,
                 event->phy_updated.tx_phy, event->phy_updated.rx_phy, event->phy_updated.status);
        return 0;

    case BLE_GAP_EVENT_DATA_LEN_CHG:
        /* NimBLE (ESP-IDF 6.0.1) leaves data_len_chg.conn_handle at 0, so no link number here. */
        ESP_LOGI(TAG, "link data length: tx %u octets / %u us, rx %u octets / %u us",
                 event->data_len_chg.max_tx_octets, event->data_len_chg.max_tx_time,
                 event->data_len_chg.max_rx_octets, event->data_len_chg.max_rx_time);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        bool secured = event->enc_change.status == 0 && link_secured(event->enc_change.conn_handle);
        struct ble_gap_conn_desc ed;
        bool have_id = ble_gap_conn_find(event->enc_change.conn_handle, &ed) == 0;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find(event->enc_change.conn_handle);
        if (c) c->secured = secured;
        /* Once per link: a second data-length request while the first is unanswered stalls the
         * controller's HCI for seconds and NimBLE resets (seen with an Android 9 phone). */
        bool want_dle = secured && s_tx_octets_cap && c && !c->dle_set;
        uint32_t dle_arg = 0;
        if (want_dle) {
            c->dle_set = true;
            c->full_size = have_id && full_index(&ed.peer_id_addr) >= 0;
            dle_arg = ((uint32_t)c->gen << 16) | event->enc_change.conn_handle;
        }
        bool stored_full = want_dle && c->full_size;
        xSemaphoreGive(s_lock);
        if (stored_full) {
            ESP_LOGI(TAG, "conn %u: this phone keeps full-size link packets (stored setting)",
                     event->enc_change.conn_handle);
        }
        if (want_dle) {
            xTaskCreate(settled_dle_task, "ble_dle", 2560, (void *)(uintptr_t)dle_arg, 5, NULL);
        }
        if (s_passkey_active) {
            s_passkey_active = false;
            if (secured) {
                ESP_LOGI(TAG, "paired");
                close_pairing();
            }
        }
        if (!secured) {
            ESP_LOGW(TAG, "link security failed (%d)", event->enc_change.status);
            ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_AUTH_FAIL);
        }
        return 0;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        uint16_t h = event->passkey.conn_handle;
        if (event->passkey.params.action != BLE_SM_IOACT_DISP || !pairing_open()) {
            ESP_LOGW(TAG, "pairing request refused (pairing window closed)");
            ble_gap_terminate(h, BLE_ERR_AUTH_FAIL);
            return 0;
        }
        struct ble_sm_io pk = {.action = BLE_SM_IOACT_DISP, .passkey = esp_random() % 1000000};
        s_passkey = pk.passkey;
        s_passkey_active = true;
        ble_sm_inject_io(h, &pk);
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Only replace an existing bond while the owner has pairing open. */
        if (!pairing_open()) return BLE_GAP_REPEAT_PAIRING_IGNORE;
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &d) == 0) {
            ble_store_util_delete_peer(&d.peer_id_addr);
            full_forget(&d.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_SUBSCRIBE: {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find(event->subscribe.conn_handle);
        if (c) {
            if (event->subscribe.attr_handle == h_events) c->sub_events = event->subscribe.cur_notify;
            if (event->subscribe.attr_handle == h_records) c->sub_records = event->subscribe.cur_notify;
        }
        xSemaphoreGive(s_lock);
        if (event->subscribe.attr_handle == h_records && !event->subscribe.cur_notify) {
            edge_stream_link_down(event->subscribe.conn_handle);
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU: {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        conn_t *c = conn_find(event->mtu.conn_handle);
        if (c) c->mtu = event->mtu.value;
        xSemaphoreGive(s_lock);
        return 0;
    }

    default:
        return 0;
    }
}

/* ---- Host ------------------------------------------------------------------------ */

static void on_sync(void)
{
    int privacy = 0;
#if CONFIG_EDGE_BLE_PRIVACY
    privacy = 1;
#endif
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(privacy, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "no usable address: %d", rc);
        return;
    }
    s_synced = true;
    s_fast_until = edge_uptime_ms() + FAST_ADV_MS;
    s_tx_octets_cap = BOOT_TX_OCTETS;
    write_default_data_len(s_tx_octets_cap);
    if (bond_count() == 0) s_pair_until = FOREVER; /* first use: open until a phone pairs */
    if (!s_reap_init) {
        s_reap_init = true;
        ble_npl_callout_init(&s_reap_co, nimble_port_get_dflt_eventq(), reap_cb, NULL);
    }
    memset(s_unrep, 0, sizeof(s_unrep));
    ble_npl_callout_reset(&s_reap_co, ble_npl_time_ms_to_ticks32(1000));
    advertise();
    scan_apply();
}

static void on_reset(int reason)
{
    s_synced = false;
    s_scanning = false;
    ESP_LOGW(TAG, "host reset: %d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t edge_ble_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_rec_lock = xSemaphoreCreateMutex();
    full_load();
    snprintf(s_name, sizeof(s_name), "Sotreus Edge %s", edge_identity()->suffix);

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble init: %s", esp_err_to_name(err));
        return err;
    }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT setup failed: %d", rc);
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set("Sotreus Edge");
    ble_store_config_init();

    nimble_port_freertos_init(host_task);

    esp_timer_create_args_t ta = {.callback = tick_cb, .name = "ble_tick"};
    esp_timer_create(&ta, &s_tick);
    esp_timer_start_periodic(s_tick, 1000 * 1000);
    return ESP_OK;
}

void edge_ble_get_info(edge_ble_info_t *o)
{
    memset(o, 0, sizeof(*o));
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_CONN; i++) {
        if (!s_conn[i].used) continue;
        o->connections++;
        o->secured += s_conn[i].secured;
    }
    xSemaphoreGive(s_lock);
    o->bonds = s_synced ? bond_count() : 0;
    o->pairing_open = pairing_open();
    if (o->pairing_open && s_pair_until != FOREVER) {
        o->pairing_left_s = (uint32_t)((s_pair_until - edge_uptime_ms()) / 1000);
    }
    o->passkey_active = s_passkey_active;
    o->passkey = s_passkey;
    o->scanning = s_scanning;
    o->scan_active = s_scan_active_mode;
}
