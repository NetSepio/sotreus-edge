// Battery voltage via the Heltec V3 divider on GPIO1 (enabled through GPIO37).
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool present;    /* a plausible LiPo voltage was measured */
    uint32_t mv;
    int percent;     /* rough estimate from an open-circuit LiPo curve */
} edge_battery_t;

void edge_battery_init(void);
/** Takes a fresh measurement (a few ms). */
void edge_battery_update(void);
void edge_battery_get(edge_battery_t *out);
