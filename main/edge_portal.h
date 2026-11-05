// Captive portal for the Edge's own access point.
//
// A device that joins Sotreus-Edge-XXXX gets 192.168.4.1 as its DNS server.
// Until that device has logged in to the setup page, every name resolves to
// 192.168.4.1 and the operating system's connectivity check is redirected to
// the setup page, so the phone or laptop opens it as a sign-in sheet.
//
// Once a device has logged in, its lookups answer "no such name". The OS then
// treats the network as "connected, no internet": it stops prompting and
// keeps using mobile data for internet, which the Sotreus app relies on.
//
// The DNS responder is bound to the access point address only, so it never
// answers on a network the Edge has joined as a station.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define EDGE_PORTAL_IP      "192.168.4.1"

void edge_portal_start(void);

/** Marks a client (IPv4, network byte order) as done with the portal. */
void edge_portal_client_done(uint32_t ip);
bool edge_portal_is_done(uint32_t ip);
/** Forgets every client (after a PIN change or logout of all sessions). */
void edge_portal_reset(void);
