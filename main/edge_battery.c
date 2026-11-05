#include "edge_battery.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "board.h"

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static int s_ctrl_level = -1; /* level on GPIO37 that enables the divider */
static edge_battery_t s_last;

static int read_mv(int ctrl_level)
{
    gpio_set_direction(PIN_VBAT_CTRL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_VBAT_CTRL, ctrl_level);
    vTaskDelay(pdMS_TO_TICKS(5));
    int sum = 0, n = 0;
    for (int i = 0; i < 8; i++) {
        int raw, mv;
        if (adc_oneshot_read(s_adc, ADC_CHANNEL_0, &raw) != ESP_OK) continue;
        if (s_cali && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            sum += mv;
        } else {
            sum += raw * 1250 / 4095;
        }
        n++;
    }
    /* Release the pin so the divider does not drain the battery. */
    gpio_set_direction(PIN_VBAT_CTRL, GPIO_MODE_INPUT);
    if (!n) return 0;
    return (int)((int64_t)(sum / n) * CONFIG_EDGE_BATT_DIVIDER_X1000 / 1000);
}

static int lipo_percent(uint32_t mv)
{
    static const uint16_t pts[][2] = {{4200, 100}, {4100, 90}, {4000, 78}, {3900, 62}, {3800, 45},
                                      {3700, 25}, {3600, 10}, {3500, 4}, {3300, 0}};
    if (mv >= pts[0][0]) return 100;
    for (int i = 1; i < 9; i++) {
        if (mv >= pts[i][0]) {
            int dv = pts[i - 1][0] - pts[i][0];
            int dp = pts[i - 1][1] - pts[i][1];
            return pts[i][1] + (int)(mv - pts[i][0]) * dp / dv;
        }
    }
    return 0;
}

void edge_battery_init(void)
{
    adc_oneshot_unit_init_cfg_t uc = {.unit_id = ADC_UNIT_1};
    if (adc_oneshot_new_unit(&uc, &s_adc) != ESP_OK) {
        ESP_LOGW(TAG, "ADC unavailable");
        return;
    }
    adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_2_5, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &cc);
    adc_cali_curve_fitting_config_t cf = {.unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_0,
                                          .atten = ADC_ATTEN_DB_2_5, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_cali_create_scheme_curve_fitting(&cf, &s_cali) != ESP_OK) s_cali = NULL;

#if CONFIG_EDGE_BATT_CTRL_LOW
    s_ctrl_level = 0;
#elif CONFIG_EDGE_BATT_CTRL_HIGH
    s_ctrl_level = 1;
#else
    /* Board revisions differ in the enable polarity: keep whichever reads a voltage. */
    int lo = read_mv(0);
    int hi = read_mv(1);
    s_ctrl_level = hi > lo + 300 ? 1 : 0;
    ESP_LOGI(TAG, "divider enable level %d (low %d mV, high %d mV)", s_ctrl_level, lo, hi);
#endif
    edge_battery_update();
}

/* Recent readings: a cell is steady, a charger with no cell behind it wanders. */
#define HIST 6
static int s_hist[HIST];
static int s_hist_n;

void edge_battery_update(void)
{
    if (!s_adc || s_ctrl_level < 0) return;
    int lo = 100000, hi = 0, sum = 0;
    for (int i = 0; i < 4; i++) {
        int mv = read_mv(s_ctrl_level);
        if (mv < lo) lo = mv;
        if (mv > hi) hi = mv;
        sum += mv;
        if (i < 3) vTaskDelay(pdMS_TO_TICKS(40));
    }
    int mv = sum / 4;
    memmove(&s_hist[1], &s_hist[0], (HIST - 1) * sizeof(int));
    s_hist[0] = mv;
    if (s_hist_n < HIST) s_hist_n++;
    for (int i = 0; i < s_hist_n; i++) {
        if (s_hist[i] < lo) lo = s_hist[i];
        if (s_hist[i] > hi) hi = s_hist[i];
    }
    s_last.mv = (uint32_t)(mv < 0 ? 0 : mv);
    /* Above 4.25 V no LiPo is connected (the charger stops at 4.2 V); a spread of
     * more than ~150 mV between readings is the charger output, not a cell. */
    bool external = hi > 4250 || (hi - lo) > 150;
    s_last.external = external && mv > 3000;
    s_last.present = !external && mv >= 3000 && mv <= 4250;
    s_last.percent = s_last.present ? lipo_percent(s_last.mv) : 0;
}

void edge_battery_get(edge_battery_t *out)
{
    *out = s_last;
}
