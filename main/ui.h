// On-device UI: OLED screens, the PRG button and the LED.
//
// PRG short press   next screen (and wakes the display)
// PRG hold 3-8 s    open Bluetooth pairing for 2 minutes
// PRG hold 10 s     erase settings, pairings and records (counts down from 5 s)
#pragma once

void ui_init(void);

/** Shows the restart screen until the Edge restarts. to_mode: "MESH" / "AWARE", or NULL. */
void ui_show_restart(const char *to_mode);
