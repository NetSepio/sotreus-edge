// Wall-clock handling. The Edge has no RTC battery and makes no outbound
// connections, so wall time is only known after a paired phone or the setup
// page sends it. Until then every timestamp is uptime and flagged as such.
#pragma once

#include <stdbool.h>
#include <stdint.h>

void edge_time_init(void);
int64_t edge_uptime_ms(void);
bool edge_time_valid(void);
/** source: "phone" | "browser" | "api". */
void edge_time_set_unix_ms(int64_t unix_ms, const char *source);
/** Converts an uptime instant to unix ms; returns -1 when wall time is unknown. */
int64_t edge_time_to_unix_ms(int64_t uptime_ms);
const char *edge_time_source(void);
/** Uptime at which wall time was last set, -1 if never. */
int64_t edge_time_set_at_uptime_ms(void);
