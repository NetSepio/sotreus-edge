// Captive-portal DNS answers (pure C, host-tested).
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Turns the DNS query in p[0..len) into a reply, in place. A queries are
 * answered with 192.168.4.1 (TTL 1 s); other types get an empty NOERROR.
 * With client_done every name is NXDOMAIN. Returns the reply length, or 0
 * when the packet should be ignored.
 */
int edge_dns_answer(uint8_t *p, int len, int cap, bool client_done);
