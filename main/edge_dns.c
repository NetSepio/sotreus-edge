#include "edge_dns.h"

#include <string.h>

/* Builds the reply in place. Returns its length, or 0 to stay silent. */
int edge_dns_answer(uint8_t *p, int len, int cap, bool client_done)
{
    if (len < 12) return 0;
    uint16_t flags = (uint16_t)((p[2] << 8) | p[3]);
    if (flags & 0x8000) return 0;                 /* a response, not a query */
    if (((flags >> 11) & 0x0f) != 0) return 0;    /* only standard queries */
    uint16_t qd = (uint16_t)((p[4] << 8) | p[5]);
    if (qd != 1) return 0;

    /* Walk the question name. */
    int i = 12;
    while (i < len && p[i] != 0) {
        if ((p[i] & 0xc0) != 0) return 0;         /* no compression in questions */
        i += p[i] + 1;
    }
    if (i + 5 > len) return 0;
    i++;                                          /* terminating zero */
    uint16_t qtype = (uint16_t)((p[i] << 8) | p[i + 1]);
    uint16_t qclass = (uint16_t)((p[i + 2] << 8) | p[i + 3]);
    int qend = i + 4;

    /* Header: response, authoritative, recursion desired copied, no recursion available. */
    p[2] = (uint8_t)(0x84 | (p[2] & 0x01));
    p[3] = 0x00;
    p[6] = p[7] = 0;                              /* answers */
    p[8] = p[9] = 0;                              /* authority */
    p[10] = p[11] = 0;                            /* additional (drops EDNS) */
    int out = qend;

    if (client_done) {
        p[3] = 0x03;                              /* NXDOMAIN: "no internet", no prompt */
        return out;
    }
    if (qtype == 1 && qclass == 1 && out + 16 <= cap) {
        static const uint8_t ans[] = {
            0xc0, 0x0c,             /* name: pointer to the question */
            0x00, 0x01, 0x00, 0x01, /* type A, class IN */
            0x00, 0x00, 0x00, 0x01, /* TTL 1 s, so logging in takes effect quickly */
            0x00, 0x04,             /* 4 bytes */
            192, 168, 4, 1,
        };
        memcpy(p + out, ans, sizeof(ans));
        out += sizeof(ans);
        p[7] = 1;
    }
    /* Other types (AAAA, HTTPS, ...): empty NOERROR so clients fall back to IPv4. */
    return out;
}
