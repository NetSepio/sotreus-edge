#include "edge_time.h"

#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "edge_util.h"

static const char *TAG = "time";

/* 2024-01-01T00:00:00Z: anything earlier is rejected as not a real clock. */
#define MIN_PLAUSIBLE_UNIX_MS 1704067200000LL

static volatile int64_t s_offset_ms; /* unix_ms - uptime_ms */
static volatile bool s_valid;
static volatile int64_t s_set_at = -1;
static char s_source[12] = "none";

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
