#include "edge_console.h"

#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "edge_api.h"

static const char *TAG = "console";

#define LINE_MAX_LEN 2048

static void console_task(void *arg)
{
    char *line = malloc(LINE_MAX_LEN);
    size_t len = 0;
    uint8_t buf[128];
    for (;;) {
        int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), pdMS_TO_TICKS(200));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\r') continue;
            if (c != '\n') {
                if (len < LINE_MAX_LEN - 1) line[len++] = c;
                continue;
            }
            line[len] = '\0';
            if (len && line[0] == '{') {
                char *copy = strdup(line);
                if (copy) edge_api_submit(EDGE_SINK_UART, 0, copy);
            }
            len = 0;
        }
    }
}

void edge_console_init(void)
{
    esp_err_t err = uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "console input unavailable: %s", esp_err_to_name(err));
        return;
    }
    xTaskCreate(console_task, "console", 3072, NULL, 3, NULL);
}
