#include "edge_time.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"

#include "edge_config.h"

#include "edge_util.h"

static const char *TAG = "time";

/* 2024-01-01T00:00:00Z: anything earlier is rejected as not a real clock. */
#define MIN_PLAUSIBLE_UNIX_MS 1704067200000LL

static volatile int64_t s_offset_ms; /* unix_ms - uptime_ms */
static volatile bool s_valid;
static volatile int64_t s_set_at = -1;
static char s_source[12] = "none";
static volatile bool s_first_pending;
static bool s_tz_known;
static int s_tz_min;

void edge_time_init(void)
{
    s_valid = false;
}

int64_t edge_uptime_ms(void)
{
    return esp_timer_get_time() / 1000;
}

bool edge_time_valid(void)
{
    return s_valid;
}

void edge_time_set_unix_ms(int64_t unix_ms, const char *source)
{
    if (unix_ms < MIN_PLAUSIBLE_UNIX_MS) {
        ESP_LOGW(TAG, "ignoring implausible time %lld", (long long)unix_ms);
        return;
    }
    int64_t up = edge_uptime_ms();
    if (!s_valid) s_first_pending = true;
    s_offset_ms = unix_ms - up;
    s_set_at = up;
    s_valid = true;
    edge_strlcpy(s_source, source ? source : "api", sizeof(s_source));
    struct timeval tv = {.tv_sec = unix_ms / 1000, .tv_usec = (unix_ms % 1000) * 1000};
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "wall time set from %s", s_source);
}

int64_t edge_time_to_unix_ms(int64_t uptime_ms)
{
    if (!s_valid) return -1;
    return uptime_ms + s_offset_ms;
}

const char *edge_time_source(void)
{
    return s_source;
}

int64_t edge_time_set_at_uptime_ms(void)
{
    return s_set_at;
}

bool edge_time_take_first_set(void)
{
    if (!s_first_pending) return false;
    s_first_pending = false;
    return true;
}

static void sntp_synced(struct timeval *tv)
{
    edge_time_set_unix_ms((int64_t)tv->tv_sec * 1000 + tv->tv_usec / 1000, "ntp");
}

void edge_time_start_sntp(void)
{
    static bool started;
    if (started) return;
    started = true;
    esp_sntp_config_t c = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    c.sync_cb = sntp_synced;
    if (esp_netif_sntp_init(&c) != ESP_OK) {
        started = false;
        ESP_LOGW(TAG, "SNTP did not start");
    }
}

void edge_time_load_tz(void)
{
    int32_t m;
    if (edge_tz_get(&m)) {
        s_tz_min = m;
        s_tz_known = true;
    }
}

void edge_time_set_tz_offset_min(int minutes)
{
    if (minutes < -14 * 60 || minutes > 14 * 60) return;
    if (s_tz_known && s_tz_min == minutes) return;
    s_tz_min = minutes;
    s_tz_known = true;
    edge_tz_set(minutes);
}

bool edge_time_tz_known(void)
{
    return s_tz_known;
}

int edge_time_tz_offset_min(void)
{
    return s_tz_min;
}

bool edge_time_local_hhmm(char *out, size_t cap)
{
    int64_t u = edge_time_to_unix_ms(edge_uptime_ms());
    if (u < 0) return false;
    time_t t = (time_t)(u / 1000) + (s_tz_known ? s_tz_min * 60 : 0);
    struct tm tm;
    gmtime_r(&t, &tm);
    snprintf(out, cap, s_tz_known ? "%02d:%02d" : "%02d:%02d UTC", tm.tm_hour, tm.tm_min);
    return true;
}
