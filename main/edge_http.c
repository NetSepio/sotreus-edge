#include "edge_http.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "psa/crypto.h"

#include "edge_api.h"
#include "edge_app.h"
#include "edge_config.h"
#include "edge_log.h"
#include "edge_portal.h"
#include "edge_records.h"
#include "edge_time.h"
#include "edge_util.h"

static const char *TAG = "http";

/* Embedded web assets (see main/CMakeLists.txt). */
#define ASSET(sym) \
    extern const uint8_t sym##_start[] asm("_binary_" #sym "_start"); \
    extern const uint8_t sym##_end[] asm("_binary_" #sym "_end");
ASSET(index_html_gz)
ASSET(maplibre_gl_4_7_1_js_gz)
ASSET(maplibre_gl_4_7_1_css_gz)
ASSET(uplot_1_6_31_min_js_gz)
ASSET(uplot_1_6_31_min_css_gz)
ASSET(satellite_5_0_0_min_js_gz)
ASSET(geist_regular_woff2)
ASSET(geist_medium_woff2)
ASSET(geist_mono_regular_woff2)
ASSET(instrument_serif_regular_woff2)

typedef struct {
    const char *path;
    const uint8_t *start;
    const uint8_t *end;
    const char *type;
    bool gzip;
} asset_t;

#define A(path, sym, type, gz) {path, sym##_start, sym##_end, type, gz}
static const asset_t ASSETS[] = {
    A("/assets/maplibre-gl-4.7.1.js", maplibre_gl_4_7_1_js_gz, "text/javascript", true),
    A("/assets/maplibre-gl-4.7.1.css", maplibre_gl_4_7_1_css_gz, "text/css", true),
    A("/assets/uplot-1.6.31.min.js", uplot_1_6_31_min_js_gz, "text/javascript", true),
    A("/assets/uplot-1.6.31.min.css", uplot_1_6_31_min_css_gz, "text/css", true),
    A("/assets/satellite-5.0.0.min.js", satellite_5_0_0_min_js_gz, "text/javascript", true),
    A("/assets/geist-regular.woff2", geist_regular_woff2, "font/woff2", false),
    A("/assets/geist-medium.woff2", geist_medium_woff2, "font/woff2", false),
    A("/assets/geist-mono-regular.woff2", geist_mono_regular_woff2, "font/woff2", false),
    A("/assets/instrument-serif-regular.woff2", instrument_serif_regular_woff2, "font/woff2", false),
};

#define MAX_SESSIONS    4
#define MAX_WS          4
#define TOKEN_LEN       32
#define MAX_BODY        4096

static httpd_handle_t s_server;
static SemaphoreHandle_t s_lock;
static char s_tokens[MAX_SESSIONS][TOKEN_LEN + 1];
static int s_token_next;
static char s_pin_seen[9];
static int s_ws_fds[MAX_WS];
static int s_fail_count;
static int64_t s_locked_until;

/* ---- Sessions ------------------------------------------------------------------ */

static void sessions_check_pin(void)
{
    /* A new PIN invalidates every session issued under the old one. */
    const char *pin = edge_identity()->access_pin;
    if (strcmp(pin, s_pin_seen) != 0) {
        memset(s_tokens, 0, sizeof(s_tokens));
        for (int i = 0; i < MAX_WS; i++) s_ws_fds[i] = -1;
        edge_strlcpy(s_pin_seen, pin, sizeof(s_pin_seen));
        edge_portal_reset();
    }
}

static bool token_valid(const char *t)
{
    if (!t || strlen(t) != TOKEN_LEN) return false;
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    sessions_check_pin();
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (!s_tokens[i][0]) continue;
        /* constant-time compare */
        uint8_t diff = 0;
        for (int k = 0; k < TOKEN_LEN; k++) diff |= (uint8_t)(s_tokens[i][k] ^ t[k]);
        ok |= diff == 0;
    }
    xSemaphoreGive(s_lock);
    return ok;
}

static void new_token(char out[TOKEN_LEN + 1])
{
    uint8_t raw[TOKEN_LEN / 2];
    esp_fill_random(raw, sizeof(raw));
    edge_hex(raw, sizeof(raw), out);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    sessions_check_pin();
    memcpy(s_tokens[s_token_next], out, TOKEN_LEN + 1);
    s_token_next = (s_token_next + 1) % MAX_SESSIONS;
    xSemaphoreGive(s_lock);
}

void edge_http_logout_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_tokens, 0, sizeof(s_tokens));
    for (int i = 0; i < MAX_WS; i++) s_ws_fds[i] = -1;
    xSemaphoreGive(s_lock);
    edge_portal_reset();
}

static bool authorized(httpd_req_t *req)
{
    char h[64];
    if (httpd_req_get_hdr_value_str(req, "Authorization", h, sizeof(h)) != ESP_OK) return false;
    if (strncmp(h, "Bearer ", 7) != 0) return false;
    return token_valid(h + 7);
}

/* ---- Captive portal helpers -------------------------------------------------------- */

/* IPv4 address (network order) of the peer or of our end of the socket. */
static uint32_t sock_ipv4(int fd, bool peer)
{
    struct sockaddr_storage ss;
    socklen_t l = sizeof(ss);
    int r = peer ? getpeername(fd, (struct sockaddr *)&ss, &l) : getsockname(fd, (struct sockaddr *)&ss, &l);
    if (r != 0) return 0;
    if (ss.ss_family == AF_INET) return ((struct sockaddr_in *)&ss)->sin_addr.s_addr;
    if (ss.ss_family == AF_INET6) {
        uint32_t v; /* IPv4-mapped IPv6 address: last four bytes */
        memcpy(&v, ((uint8_t *)&((struct sockaddr_in6 *)&ss)->sin6_addr) + 12, 4);
        return v;
    }
    return 0;
}

/* True when the request came in on the Edge's own access point. */
static bool via_ap(httpd_req_t *req)
{
    return sock_ipv4(httpd_req_to_sockfd(req), false) == inet_addr(EDGE_PORTAL_IP);
}

static esp_err_t redirect_to_portal(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" EDGE_PORTAL_IP "/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, "<a href=\"http://" EDGE_PORTAL_IP "/\">Sotreus Edge setup</a>");
}

/* Unknown paths, including every OS connectivity check (generate_204,
 * hotspot-detect.html, connecttest.txt, ...): open the setup page for devices
 * on the access point that have not logged in yet; plain 404 otherwise. */
static esp_err_t h_not_found(httpd_req_t *req, httpd_err_code_t err)
{
    if (via_ap(req) && !edge_portal_is_done(sock_ipv4(httpd_req_to_sockfd(req), true))) {
        return redirect_to_portal(req);
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
    return ESP_FAIL;
}

/* ---- Responses ----------------------------------------------------------------- */

static void common_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
    httpd_resp_set_hdr(req, "Referrer-Policy", "no-referrer");
}

static esp_err_t send_json(httpd_req_t *req, const char *status, const cJSON *body)
{
    common_headers(req);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    char *s = cJSON_PrintUnformatted(body);
    esp_err_t err = httpd_resp_sendstr(req, s ? s : "{}");
    free(s);
    return err;
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *msg)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", false);
    cJSON_AddStringToObject(o, "error", msg);
    esp_err_t err = send_json(req, status, o);
    cJSON_Delete(o);
    return err;
}

static cJSON *read_json_body(httpd_req_t *req)
{
    if (req->content_len == 0 || req->content_len > MAX_BODY) return NULL;
    char *buf = malloc(req->content_len + 1);
    if (!buf) return NULL;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            free(buf);
            return NULL;
        }
        got += (size_t)r;
    }
    buf[got] = '\0';
    cJSON *j = cJSON_Parse(buf);
    free(buf);
    return j;
}

/* ---- Handlers ------------------------------------------------------------------ */

static esp_err_t h_index(httpd_req_t *req)
{
    /* On the access point every name resolves to the Edge: move other host
     * names to the canonical address so the page always has one origin. */
    char host[64];
    if (via_ap(req) && httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) == ESP_OK &&
        strcmp(host, EDGE_PORTAL_IP) != 0 && !strstr(host, ".local")) {
        return redirect_to_portal(req);
    }
    common_headers(req);
    httpd_resp_set_hdr(req, "Content-Security-Policy",
                       "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; "
                       "font-src 'self'; img-src 'self' data: blob: https:; connect-src 'self' ws: wss: https:; "
                       "worker-src 'self' blob:; child-src blob:; frame-ancestors 'none'");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
}

/* Versioned file names, so they can be cached for a year. */
static esp_err_t h_asset(httpd_req_t *req)
{
    for (size_t i = 0; i < sizeof(ASSETS) / sizeof(ASSETS[0]); i++) {
        const asset_t *a = &ASSETS[i];
        if (strcmp(req->uri, a->path) != 0) continue;
        httpd_resp_set_type(req, a->type);
        httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000, immutable");
        httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
        if (a->gzip) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)a->start, a->end - a->start);
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
    return ESP_FAIL;
}


static esp_err_t h_no_content(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t h_login(httpd_req_t *req)
{
    int64_t t = edge_uptime_ms();
    if (t < s_locked_until) return send_error(req, "429 Too Many Requests", "too many attempts, wait a minute");
    cJSON *j = read_json_body(req);
    const cJSON *pin = j ? cJSON_GetObjectItemCaseSensitive(j, "pin") : NULL;
    const char *want = edge_identity()->access_pin;
    bool ok = cJSON_IsString(pin) && strlen(pin->valuestring) == strlen(want);
    if (ok) {
        uint8_t diff = 0;
        for (size_t i = 0; i < strlen(want); i++) diff |= (uint8_t)(pin->valuestring[i] ^ want[i]);
        ok = diff == 0;
    }
    if (!ok) {
        if (++s_fail_count >= 5) {
            s_fail_count = 0;
            s_locked_until = t + 60000;
        }
        cJSON_Delete(j);
        return send_error(req, "401 Unauthorized", "wrong PIN");
    }
    s_fail_count = 0;
    if (via_ap(req)) edge_portal_client_done(sock_ipv4(httpd_req_to_sockfd(req), true));
    /* The browser's clock is a reasonable source when nothing better has set the time. */
    const cJSON *ms = cJSON_GetObjectItemCaseSensitive(j, "unix_ms");
    if (!edge_time_valid() && cJSON_IsNumber(ms)) {
        edge_time_set_unix_ms((int64_t)ms->valuedouble, "browser");
    }
    const cJSON *tz = cJSON_GetObjectItemCaseSensitive(j, "tz_offset_min");
    if (cJSON_IsNumber(tz)) edge_time_set_tz_offset_min((int)tz->valuedouble);
    cJSON_Delete(j);
    char tok[TOKEN_LEN + 1];
    new_token(tok);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", true);
    cJSON_AddStringToObject(o, "token", tok);
    esp_err_t err = send_json(req, "200 OK", o);
    cJSON_Delete(o);
    return err;
}

static esp_err_t h_logout(httpd_req_t *req)
{
    char h[64];
    if (httpd_req_get_hdr_value_str(req, "Authorization", h, sizeof(h)) == ESP_OK && strncmp(h, "Bearer ", 7) == 0) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int i = 0; i < MAX_SESSIONS; i++) {
            if (strcmp(s_tokens[i], h + 7) == 0) s_tokens[i][0] = '\0';
        }
        xSemaphoreGive(s_lock);
    }
    return h_no_content(req);
}

static esp_err_t h_cmd(httpd_req_t *req)
{
    if (!authorized(req)) return send_error(req, "401 Unauthorized", "log in first");
    cJSON *j = read_json_body(req);
    if (!cJSON_IsObject(j)) {
        cJSON_Delete(j);
        return send_error(req, "400 Bad Request", "body must be a JSON object up to 4 KB");
    }
    cJSON *resp = edge_api_handle(j, EDGE_SINK_WS, -1);
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(j, "id");
    if (id) cJSON_AddItemToObject(resp, "id", cJSON_Duplicate(id, true));
    esp_err_t err = send_json(req, "200 OK", resp);
    cJSON_Delete(resp);
    cJSON_Delete(j);
    return err;
}

static esp_err_t h_records(httpd_req_t *req)
{
    if (!authorized(req)) return send_error(req, "401 Unauthorized", "log in first");
    char q[96] = "", v[16];
    uint32_t from = 0, limit = 500;
    bool bin = false;
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "from", v, sizeof(v)) == ESP_OK) from = (uint32_t)strtoul(v, NULL, 10);
        if (httpd_query_key_value(q, "limit", v, sizeof(v)) == ESP_OK) limit = (uint32_t)strtoul(v, NULL, 10);
        if (httpd_query_key_value(q, "fmt", v, sizeof(v)) == ESP_OK) bin = strcmp(v, "bin") == 0;
    }
    if (limit == 0 || limit > 5000) limit = 5000;

    common_headers(req);
    httpd_resp_set_type(req, bin ? "application/octet-stream" : "application/x-ndjson");
    edge_log_stats_t ls;
    edge_log_stats(&ls);
    char hv[16];
    snprintf(hv, sizeof(hv), "%lu", (unsigned long)ls.head_seq);
    httpd_resp_set_hdr(req, "X-Edge-Head", hv);
    char ov[16];
    snprintf(ov, sizeof(ov), "%lu", (unsigned long)ls.oldest_seq);
    httpd_resp_set_hdr(req, "X-Edge-Oldest", ov);

    const int batch = 8;
    edge_rec_t *recs = malloc(batch * sizeof(edge_rec_t));
    if (!recs) return send_error(req, "500 Internal Server Error", "no memory");
    uint32_t sent = 0, cursor = from;
    esp_err_t err = ESP_OK;
    while (sent < limit && err == ESP_OK) {
        int want = (int)((limit - sent) < (uint32_t)batch ? (limit - sent) : (uint32_t)batch);
        int n = edge_log_read_from(cursor, recs, want);
        if (n <= 0) break;
        for (int i = 0; i < n && err == ESP_OK; i++) {
            if (bin) {
                err = httpd_resp_send_chunk(req, (const char *)&recs[i], sizeof(edge_rec_t));
            } else {
                cJSON *o = edge_rec_to_json(&recs[i]);
                char *s = cJSON_PrintUnformatted(o);
                cJSON_Delete(o);
                if (s) {
                    err = httpd_resp_send_chunk(req, s, strlen(s));
                    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\n", 1);
                    free(s);
                }
            }
            cursor = recs[i].seq + 1;
            sent++;
        }
    }
    free(recs);
    if (err == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

static esp_err_t h_ota(httpd_req_t *req)
{
    if (!authorized(req)) return send_error(req, "401 Unauthorized", "log in first");
    if (req->content_len < 1024) return send_error(req, "400 Bad Request", "firmware image expected");
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part || req->content_len > part->size) return send_error(req, "400 Bad Request", "image does not fit");

    char want_hex[65] = "";
    httpd_req_get_hdr_value_str(req, "X-SHA256", want_hex, sizeof(want_hex));

    esp_ota_handle_t ota;
    if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "could not start update");
    }
    psa_crypto_init();
    psa_hash_operation_t hash = PSA_HASH_OPERATION_INIT;
    psa_hash_setup(&hash, PSA_ALG_SHA_256);

    char *buf = malloc(4096);
    size_t left = req->content_len;
    esp_err_t err = buf ? ESP_OK : ESP_ERR_NO_MEM;
    while (left > 0 && err == ESP_OK) {
        int r = httpd_req_recv(req, buf, left < 4096 ? left : 4096);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            err = ESP_FAIL;
            break;
        }
        psa_hash_update(&hash, (const uint8_t *)buf, (size_t)r);
        err = esp_ota_write(ota, buf, (size_t)r);
        left -= (size_t)r;
    }
    free(buf);
    uint8_t digest[32];
    size_t dlen = 0;
    psa_hash_finish(&hash, digest, sizeof(digest), &dlen);
    if (err != ESP_OK) {
        esp_ota_abort(ota);
        return send_error(req, "500 Internal Server Error", "upload failed");
    }
    char got_hex[65];
    edge_hex(digest, 32, got_hex);
    if (want_hex[0] && strcasecmp(want_hex, got_hex) != 0) {
        esp_ota_abort(ota);
        return send_error(req, "400 Bad Request", "SHA-256 does not match");
    }
    if (esp_ota_end(ota) != ESP_OK) return send_error(req, "400 Bad Request", "image failed validation");
    if (esp_ota_set_boot_partition(part) != ESP_OK) return send_error(req, "500 Internal Server Error", "could not select image");

    edge_record_event(EDGE_EVT_FIRMWARE, got_hex);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", true);
    cJSON_AddStringToObject(o, "sha256", got_hex);
    cJSON_AddStringToObject(o, "note", "restarting into the new firmware; it rolls back if it does not start cleanly");
    send_json(req, "200 OK", o);
    cJSON_Delete(o);
    edge_app_reboot_later(1500);
    return ESP_OK;
}

/* ---- WebSocket ------------------------------------------------------------------ */

static void ws_add(int fd)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MAX_WS; i++) {
        if (s_ws_fds[i] == fd) slot = i;
    }
    for (int i = 0; i < MAX_WS && slot < 0; i++) {
        if (s_ws_fds[i] < 0) slot = i;
    }
    if (slot >= 0) s_ws_fds[slot] = fd;
    xSemaphoreGive(s_lock);
}

static void ws_remove(int fd)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS; i++) {
        if (s_ws_fds[i] == fd) s_ws_fds[i] = -1;
    }
    xSemaphoreGive(s_lock);
}

static bool ws_known(int fd)
{
    bool k = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS; i++) k |= s_ws_fds[i] == fd;
    xSemaphoreGive(s_lock);
    return k;
}

static esp_err_t h_ws(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        char q[80] = "", tok[TOKEN_LEN + 2] = "";
        if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
            httpd_query_key_value(q, "token", tok, sizeof(tok));
        }
        if (!token_valid(tok)) {
            ESP_LOGW(TAG, "WebSocket refused: no valid session");
            return ESP_FAIL; /* closes the socket */
        }
        ws_add(fd);
        return ESP_OK;
    }
    if (!ws_known(fd)) return ESP_FAIL;

    httpd_ws_frame_t f = {.type = HTTPD_WS_TYPE_TEXT};
    esp_err_t err = httpd_ws_recv_frame(req, &f, 0);
    if (err != ESP_OK) return err;
    if (f.type != HTTPD_WS_TYPE_TEXT || f.len == 0) return ESP_OK;
    if (f.len > MAX_BODY) return ESP_FAIL;
    char *buf = malloc(f.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    f.payload = (uint8_t *)buf;
    err = httpd_ws_recv_frame(req, &f, f.len);
    if (err != ESP_OK) {
        free(buf);
        return err;
    }
    buf[f.len] = '\0';
    edge_api_submit(EDGE_SINK_WS, fd, buf); /* response arrives through the event queue */
    return ESP_OK;
}

typedef struct {
    int fd;
    bool binary;
    size_t len;
    char data[];
} ws_job_t;

static void ws_send_work(void *arg)
{
    ws_job_t *j = arg;
    httpd_ws_frame_t f = {.type = j->binary ? HTTPD_WS_TYPE_BINARY : HTTPD_WS_TYPE_TEXT,
                          .payload = (uint8_t *)j->data, .len = j->len, .final = true};
    int fds[MAX_WS];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(fds, s_ws_fds, sizeof(fds));
    xSemaphoreGive(s_lock);
    for (int i = 0; i < MAX_WS; i++) {
        if (fds[i] < 0 || (j->fd >= 0 && fds[i] != j->fd)) continue;
        if (httpd_ws_get_fd_info(s_server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET ||
            httpd_ws_send_frame_async(s_server, fds[i], &f) != ESP_OK) {
            ws_remove(fds[i]);
        }
    }
    free(j);
}

void edge_http_ws_send(int fd, const char *data, size_t len, bool binary)
{
    if (!s_server || edge_http_ws_count() == 0) return;
    ws_job_t *j = malloc(sizeof(ws_job_t) + len);
    if (!j) return;
    j->fd = fd;
    j->binary = binary;
    j->len = len;
    memcpy(j->data, data, len);
    if (httpd_queue_work(s_server, ws_send_work, j) != ESP_OK) free(j);
}

int edge_http_ws_count(void)
{
    if (!s_lock) return 0;
    int n = 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS; i++) n += s_ws_fds[i] >= 0;
    xSemaphoreGive(s_lock);
    return n;
}

static void on_close(httpd_handle_t hd, int fd)
{
    ws_remove(fd);
    close(fd);
}

/* ---- Server --------------------------------------------------------------------- */

esp_err_t edge_http_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_WS; i++) s_ws_fds[i] = -1;
    edge_strlcpy(s_pin_seen, edge_identity()->access_pin, sizeof(s_pin_seen));

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 16;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.close_fn = on_close;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "server start failed: %s", esp_err_to_name(err));
        return err;
    }
    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = h_index},
        {.uri = "/index.html", .method = HTTP_GET, .handler = h_index},
        {.uri = "/favicon.ico", .method = HTTP_GET, .handler = h_no_content},
        {.uri = "/api/login", .method = HTTP_POST, .handler = h_login},
        {.uri = "/api/logout", .method = HTTP_POST, .handler = h_logout},
        {.uri = "/api/cmd", .method = HTTP_POST, .handler = h_cmd},
        {.uri = "/api/records", .method = HTTP_GET, .handler = h_records},
        {.uri = "/api/ota", .method = HTTP_POST, .handler = h_ota},
        {.uri = "/assets/*", .method = HTTP_GET, .handler = h_asset},
        {.uri = "/ws", .method = HTTP_GET, .handler = h_ws, .is_websocket = true},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) httpd_register_uri_handler(s_server, &uris[i]);
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, h_not_found);
    ESP_LOGI(TAG, "HTTP API on port %d", cfg.server_port);
    return ESP_OK;
}
