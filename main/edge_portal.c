#include "edge_portal.h"

#include "edge_dns.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

static const char *TAG = "portal";

#define MAX_DONE    8
#define DNS_PORT    53
#define DNS_MAX     512

static uint32_t s_done[MAX_DONE];
static int s_done_next;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void edge_portal_client_done(uint32_t ip)
{
    if (!ip || edge_portal_is_done(ip)) return;
    portENTER_CRITICAL(&s_mux);
    s_done[s_done_next] = ip;
    s_done_next = (s_done_next + 1) % MAX_DONE;
    portEXIT_CRITICAL(&s_mux);
}

bool edge_portal_is_done(uint32_t ip)
{
    bool found = false;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < MAX_DONE; i++) found |= ip && s_done[i] == ip;
    portEXIT_CRITICAL(&s_mux);
    return found;
}

void edge_portal_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    memset(s_done, 0, sizeof(s_done));
    portEXIT_CRITICAL(&s_mux);
}

static void dns_task(void *arg)
{
    uint8_t *buf = malloc(DNS_MAX);
    for (;;) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(DNS_PORT)};
        inet_aton(EDGE_PORTAL_IP, &addr.sin_addr);
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            /* The access point interface may not be up yet. */
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "DNS for the access point on %s:53", EDGE_PORTAL_IP);
        for (;;) {
            struct sockaddr_in from;
            socklen_t fl = sizeof(from);
            int n = recvfrom(sock, buf, DNS_MAX, 0, (struct sockaddr *)&from, &fl);
            if (n < 0) break;
            int r = edge_dns_answer(buf, n, DNS_MAX, edge_portal_is_done(from.sin_addr.s_addr));
            if (r > 0) sendto(sock, buf, r, 0, (struct sockaddr *)&from, fl);
        }
        close(sock);
    }
}

void edge_portal_start(void)
{
    xTaskCreate(dns_task, "dns", 3072, NULL, 4, NULL);
}
