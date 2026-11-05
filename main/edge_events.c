#include "edge_events.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "edge_ble.h"
#include "edge_http.h"
#include "edge_records.h"

typedef struct {
    uint8_t sink;
    int target;
    size_t len;
    char *line;
} ev_item_t;

static QueueHandle_t s_q;
static volatile uint32_t s_dropped;

static void events_task(void *arg)
{
    ev_item_t it;
    for (;;) {
        if (xQueueReceive(s_q, &it, portMAX_DELAY) != pdTRUE) continue;
        switch (it.sink) {
        case EDGE_SINK_ALL:
            edge_ble_send_line(-1, it.line, it.len);
            edge_http_ws_send(-1, it.line, it.len);
            break;
        case EDGE_SINK_BLE:
            edge_ble_send_line(it.target, it.line, it.len);
            break;
        case EDGE_SINK_WS:
            edge_http_ws_send(it.target, it.line, it.len);
            break;
        case EDGE_SINK_UART:
            printf("@edge %.*s", (int)it.len, it.line);
            fflush(stdout);
            break;
        }
        free(it.line);
    }
}

void edge_events_init(void)
{
    s_q = xQueueCreate(32, sizeof(ev_item_t));
    xTaskCreate(events_task, "events", 4096, NULL, 4, NULL);
}

bool edge_events_has_listeners(void)
{
    return edge_ble_has_subscriber() || edge_http_ws_count() > 0;
}

static void enqueue(edge_sink_t sink, int target, const cJSON *msg)
{
    if (!s_q || !msg) return;
    char *s = cJSON_PrintUnformatted(msg);
    if (!s) {
        s_dropped++;
        return;
    }
    size_t len = strlen(s);
    char *line = realloc(s, len + 2);
    if (!line) {
        free(s);
        s_dropped++;
        return;
    }
    line[len] = '\n';
    line[len + 1] = '\0';
    ev_item_t it = {.sink = (uint8_t)sink, .target = target, .len = len + 1, .line = line};
    if (xQueueSend(s_q, &it, 0) != pdTRUE) {
        free(line);
        s_dropped++;
    }
}

void edge_events_publish(const cJSON *ev)
{
    if (!edge_events_has_listeners()) return;
    enqueue(EDGE_SINK_ALL, -1, ev);
}

void edge_events_publish_record(const edge_rec_t *rec)
{
    if (!edge_events_has_listeners()) return;
    cJSON *ev = cJSON_CreateObject();
    cJSON_AddStringToObject(ev, "ev", "obs");
    cJSON_AddItemToObject(ev, "rec", edge_rec_to_json(rec));
    enqueue(EDGE_SINK_ALL, -1, ev);
    cJSON_Delete(ev);
}

void edge_events_send_to(edge_sink_t sink, int target, const cJSON *msg)
{
    enqueue(sink, target, msg);
}

uint32_t edge_events_dropped(void)
{
    return s_dropped;
}

