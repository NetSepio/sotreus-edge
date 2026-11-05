// Minimal SX1262 LoRa driver (ESP-IDF SPI) for the Heltec V3 wiring:
// TCXO on DIO3, RF switch on DIO2, interrupts on DIO1.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SX_IRQ_TX_DONE          0x0001
#define SX_IRQ_RX_DONE          0x0002
#define SX_IRQ_PREAMBLE         0x0004
#define SX_IRQ_SYNC_VALID       0x0008
#define SX_IRQ_HEADER_VALID     0x0010
#define SX_IRQ_HEADER_ERR       0x0020
#define SX_IRQ_CRC_ERR          0x0040
#define SX_IRQ_TIMEOUT          0x0200
#define SX_IRQ_ALL              0x03ff

typedef struct {
    float freq_mhz;
    float bw_khz;
    uint8_t sf;
    uint8_t cr;        /* 5..8 = 4/5..4/8 */
    int8_t power_dbm;
    uint16_t preamble;
    uint8_t sync_word;
    bool rx_boosted;
} sx1262_cfg_t;

/** Sets up SPI/GPIO, resets and probes the radio. DIO1 edges notify `task`. */
esp_err_t sx1262_init(TaskHandle_t task);
esp_err_t sx1262_configure(const sx1262_cfg_t *cfg);
esp_err_t sx1262_start_rx(void);
/** Starts a transmission; completion is reported with SX_IRQ_TX_DONE. */
esp_err_t sx1262_transmit(const uint8_t *data, size_t len);
esp_err_t sx1262_standby(void);
esp_err_t sx1262_sleep(void);
uint16_t sx1262_get_irq(void);
void sx1262_clear_irq(uint16_t mask);
esp_err_t sx1262_read_packet(uint8_t *buf, size_t cap, size_t *len, int16_t *rssi, float *snr);
bool sx1262_dio1_high(void);
bool sx1262_present(void);
/** Chip mode from GetStatus: 2 STBY_RC, 3 STBY_XOSC, 4 FS, 5 RX, 6 TX (0 on error). */
uint8_t sx1262_chip_mode(void);
/** Instantaneous RSSI on the tuned channel while receiving (dBm). */
int16_t sx1262_rssi_now(void);
