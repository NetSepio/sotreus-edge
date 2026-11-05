// Application-level actions owned by main.c.
#pragma once

#include <stdint.h>

#include "edge_config.h"

/** Switches between awareness and mesh without rebooting. */
void edge_app_apply_mode(edge_mode_t mode);
/** The mode this boot runs (-1 before start-up finishes). */
int edge_app_mode(void);
/** Reboots after `delay_ms` so a response can still be delivered. */
void edge_app_reboot_later(uint32_t delay_ms);
/** Erases settings, bonds and the record log, then reboots. */
void edge_app_factory_reset_later(uint32_t delay_ms);
