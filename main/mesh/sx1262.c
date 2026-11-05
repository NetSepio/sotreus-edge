#include "sx1262.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

#include "board.h"

static const char *TAG = "sx1262";

/* Opcodes */
#define OP_SET_SLEEP            0x84
#define OP_SET_STANDBY          0x80
#define OP_SET_TX               0x83
#define OP_SET_RX               0x82
#define OP_SET_REGULATOR        0x96
#define OP_CALIBRATE            0x89
#define OP_CALIBRATE_IMAGE      0x98
#define OP_SET_PA_CONFIG        0x95
#define OP_WRITE_REG            0x0d
#define OP_READ_REG             0x1d
#define OP_WRITE_BUF            0x0e
#define OP_READ_BUF             0x1e
#define OP_SET_DIO_IRQ          0x08
#define OP_GET_IRQ              0x12
#define OP_CLEAR_IRQ            0x02
#define OP_DIO2_RF_SWITCH       0x9d
#define OP_DIO3_TCXO            0x97
#define OP_SET_RF_FREQ          0x86
#define OP_SET_PACKET_TYPE      0x8a
#define OP_SET_TX_PARAMS        0x8e
#define OP_SET_MOD_PARAMS       0x8b
#define OP_SET_PKT_PARAMS       0x8c
#define OP_SET_BUF_BASE         0x8f
#define OP_GET_RX_BUF_STATUS    0x13
#define OP_GET_PKT_STATUS       0x14
#define OP_GET_ERRORS           0x17
#define OP_CLEAR_ERRORS         0x07

/* Registers */
#define REG_SYNC_WORD_MSB       0x0740
#define REG_SYNC_WORD_LSB       0x0741
#define REG_IQ_POLARITY         0x0736
#define REG_TX_MODULATION       0x0889
#define REG_RX_GAIN             0x08ac
#define REG_TX_CLAMP            0x08d8
#define REG_OCP                 0x08e7

static spi_device_handle_t s_spi;
static TaskHandle_t s_task;
static bool s_present;
static bool s_asleep;
static uint8_t s_preamble_hi, s_preamble_lo;
static uint8_t s_txbuf[264] __attribute__((aligned(4)));
static uint8_t s_rxbuf[264] __attribute__((aligned(4)));

static void IRAM_ATTR dio1_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    if (s_task) vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static esp_err_t wait_busy(uint32_t timeout_ms)
{
    int64_t limit = (int64_t)timeout_ms * 1000;
    int64_t waited = 0;
    while (gpio_get_level(PIN_LORA_BUSY)) {
        if (waited >= limit) return ESP_ERR_TIMEOUT;
        if (waited < 2000) {
            esp_rom_delay_us(20);
            waited += 20;
        } else {
            vTaskDelay(1);
            waited += 1000;
        }
    }
    return ESP_OK;
}

static esp_err_t xfer(size_t len)
{
    if (s_asleep) {
        /* BUSY stays high in sleep; a falling NSS edge wakes the chip. */
        s_asleep = false;
        static uint8_t wake[2] __attribute__((aligned(4))) = {0xc0, 0x00}; /* GetStatus */
        spi_transaction_t w = {.length = 16, .tx_buffer = wake, .rx_buffer = NULL};
        spi_device_polling_transmit(s_spi, &w);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    esp_err_t err = wait_busy(100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "busy timeout (op 0x%02x)", s_txbuf[0]);
        return err;
    }
    spi_transaction_t t = {.length = len * 8, .tx_buffer = s_txbuf, .rx_buffer = s_rxbuf};
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t cmd(uint8_t op, const uint8_t *params, size_t n)
{
    s_txbuf[0] = op;
    if (n) memcpy(&s_txbuf[1], params, n);
    return xfer(1 + n);
}

/* Read command: opcode, `pre` bytes of parameters/NOPs, then `n` response bytes. */
static esp_err_t cmd_read(uint8_t op, const uint8_t *params, size_t np, uint8_t *out, size_t n)
{
    memset(s_txbuf, 0, 2 + np + n);
    s_txbuf[0] = op;
    if (np) memcpy(&s_txbuf[1], params, np);
    /* One status byte follows the opcode/params before the data. */
    esp_err_t err = xfer(1 + np + 1 + n);
    if (err == ESP_OK) memcpy(out, &s_rxbuf[1 + np + 1], n);
    return err;
}

static esp_err_t write_reg(uint16_t addr, uint8_t v)
{
    uint8_t p[3] = {addr >> 8, addr & 0xff, v};
    return cmd(OP_WRITE_REG, p, 3);
}

static uint8_t read_reg(uint16_t addr)
{
    uint8_t p[2] = {addr >> 8, addr & 0xff};
    uint8_t v = 0;
    /* ReadRegister: op, addr(2), status, data */
    cmd_read(OP_READ_REG, p, 2, &v, 1);
    return v;
}

static esp_err_t set_standby(void)
{
    uint8_t p = 0x00; /* STDBY_RC */
    return cmd(OP_SET_STANDBY, &p, 1);
}

bool sx1262_present(void)
{
    return s_present;
}

bool sx1262_dio1_high(void)
{
    return gpio_get_level(PIN_LORA_DIO1) != 0;
}

esp_err_t sx1262_init(TaskHandle_t task)
{
    s_task = task;

    gpio_config_t out = {.pin_bit_mask = 1ULL << PIN_LORA_RST, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&out);
    gpio_config_t in = {.pin_bit_mask = 1ULL << PIN_LORA_BUSY, .mode = GPIO_MODE_INPUT};
    gpio_config(&in);
    gpio_config_t irq = {.pin_bit_mask = 1ULL << PIN_LORA_DIO1,
                         .mode = GPIO_MODE_INPUT,
                         .pull_down_en = GPIO_PULLDOWN_ENABLE,
                         .intr_type = GPIO_INTR_POSEDGE};
    gpio_config(&irq);
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    gpio_isr_handler_add(PIN_LORA_DIO1, dio1_isr, NULL);

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LORA_MOSI,
        .miso_io_num = PIN_LORA_MISO,
        .sclk_io_num = PIN_LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(s_txbuf),
    };
    err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LORA_NSS,
        .queue_size = 1,
    };
    err = spi_bus_add_device(SPI2_HOST, &dev, &s_spi);
    if (err != ESP_OK) return err;

    /* Hardware reset. */
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (wait_busy(200) != ESP_OK) {
        ESP_LOGE(TAG, "radio not responding (BUSY stuck high)");
        return ESP_ERR_NOT_FOUND;
    }

    set_standby();
#if CONFIG_EDGE_LORA_TCXO_MV > 0
    {
        /* 1.8 V TCXO (code 0x02), 5 ms start-up = 320 * 15.625 us. */
        uint8_t v = CONFIG_EDGE_LORA_TCXO_MV >= 3300 ? 0x07 : CONFIG_EDGE_LORA_TCXO_MV >= 3000 ? 0x06
                  : CONFIG_EDGE_LORA_TCXO_MV >= 2700 ? 0x05 : CONFIG_EDGE_LORA_TCXO_MV >= 2400 ? 0x04
                  : CONFIG_EDGE_LORA_TCXO_MV >= 2200 ? 0x03 : CONFIG_EDGE_LORA_TCXO_MV >= 1800 ? 0x02
                  : CONFIG_EDGE_LORA_TCXO_MV >= 1700 ? 0x01 : 0x00;
        uint8_t p[4] = {v, 0x00, 0x01, 0x40};
        cmd(OP_DIO3_TCXO, p, 4);
        uint8_t cal = 0x7f;
        cmd(OP_CALIBRATE, &cal, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_busy(50);
        uint8_t clr[2] = {0, 0};
        cmd(OP_CLEAR_ERRORS, clr, 2); /* XOSC start error is expected after switching to TCXO */
    }
#endif
    uint8_t one = 0x01;
    cmd(OP_DIO2_RF_SWITCH, &one, 1);
    cmd(OP_SET_REGULATOR, &one, 1); /* DC-DC */
    cmd(OP_SET_PACKET_TYPE, &one, 1); /* LoRa */

    /* After reset the LoRa sync word register reads 0x14 (private network 0x1424). */
    uint8_t sw = read_reg(REG_SYNC_WORD_MSB);
    if (sw != 0x14) {
        ESP_LOGE(TAG, "unexpected sync word register 0x%02x; radio missing?", sw);
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t base[2] = {0, 0};
    cmd(OP_SET_BUF_BASE, base, 2);

    uint8_t errs[2] = {0};
    cmd_read(OP_GET_ERRORS, NULL, 0, errs, 2);
    if (errs[0] || errs[1]) ESP_LOGW(TAG, "device errors 0x%02x%02x", errs[0], errs[1]);

    s_present = true;
    ESP_LOGI(TAG, "SX1262 ready");
    return ESP_OK;
}

static uint8_t bw_code(float bw)
{
    if (bw <= 7.9f) return 0x00;
    if (bw <= 10.5f) return 0x08;
    if (bw <= 15.7f) return 0x01;
    if (bw <= 20.9f) return 0x09;
    if (bw <= 31.3f) return 0x02;
    if (bw <= 41.8f) return 0x0a;
    if (bw <= 62.6f) return 0x03;
    if (bw <= 125.1f) return 0x04;
    if (bw <= 250.1f) return 0x05;
    return 0x06;
}

static void calibrate_image(float f)
{
    uint8_t p[2];
    if (f > 900) { p[0] = 0xe1; p[1] = 0xe9; }
    else if (f > 850) { p[0] = 0xd7; p[1] = 0xdb; }
    else if (f > 770) { p[0] = 0xc1; p[1] = 0xc5; }
    else if (f > 460) { p[0] = 0x75; p[1] = 0x81; }
    else { p[0] = 0x6b; p[1] = 0x6f; }
    cmd(OP_CALIBRATE_IMAGE, p, 2);
    wait_busy(50);
}

static esp_err_t set_packet_params(uint8_t payload_len)
{
    uint8_t p[6] = {s_preamble_hi, s_preamble_lo, 0x00 /* explicit header */, payload_len,
                    0x01 /* CRC on */, 0x00 /* standard IQ */};
    return cmd(OP_SET_PKT_PARAMS, p, 6);
}

esp_err_t sx1262_configure(const sx1262_cfg_t *c)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    set_standby();

    uint32_t frf = (uint32_t)(((double)c->freq_mhz * 1e6 * (double)(1 << 25)) / 32e6);
    calibrate_image(c->freq_mhz);
    uint8_t fp[4] = {frf >> 24, frf >> 16, frf >> 8, frf};
    cmd(OP_SET_RF_FREQ, fp, 4);

    uint8_t pa[4] = {0x04, 0x07, 0x00, 0x01}; /* SX1262, up to +22 dBm */
    cmd(OP_SET_PA_CONFIG, pa, 4);
    int8_t pwr = c->power_dbm;
    if (pwr > 22) pwr = 22;
    if (pwr < -9) pwr = -9;
    uint8_t tp[2] = {(uint8_t)pwr, 0x04 /* 200 us ramp */};
    cmd(OP_SET_TX_PARAMS, tp, 2);
    write_reg(REG_OCP, 0x38); /* 140 mA */

    float tsym_ms = (float)(1 << c->sf) / c->bw_khz;
    uint8_t ldro = tsym_ms >= 16.0f ? 1 : 0;
    uint8_t mp[4] = {c->sf, bw_code(c->bw_khz), (uint8_t)(c->cr - 4), ldro};
    cmd(OP_SET_MOD_PARAMS, mp, 4);

    s_preamble_hi = c->preamble >> 8;
    s_preamble_lo = c->preamble & 0xff;
    set_packet_params(255);

    /* Errata 15.4: standard IQ needs bit 2 of 0x0736 set. */
    write_reg(REG_IQ_POLARITY, read_reg(REG_IQ_POLARITY) | 0x04);
    /* Errata 15.1: modulation quality for BW 500 kHz. */
    uint8_t txm = read_reg(REG_TX_MODULATION);
    write_reg(REG_TX_MODULATION, c->bw_khz > 400 ? (txm & ~0x04) : (txm | 0x04));
    /* Errata 15.2: better resistance to antenna mismatch. */
    write_reg(REG_TX_CLAMP, read_reg(REG_TX_CLAMP) | 0x1e);

    write_reg(REG_SYNC_WORD_MSB, (uint8_t)((c->sync_word & 0xf0) | 0x04));
    write_reg(REG_SYNC_WORD_LSB, (uint8_t)(((c->sync_word & 0x0f) << 4) | 0x04));
    write_reg(REG_RX_GAIN, 0x96); /* boosted RX gain */

    uint16_t mask = SX_IRQ_TX_DONE | SX_IRQ_RX_DONE | SX_IRQ_PREAMBLE | SX_IRQ_HEADER_VALID |
                    SX_IRQ_HEADER_ERR | SX_IRQ_CRC_ERR | SX_IRQ_TIMEOUT;
    uint8_t ip[8] = {mask >> 8, mask & 0xff, mask >> 8, mask & 0xff, 0, 0, 0, 0};
    cmd(OP_SET_DIO_IRQ, ip, 8);
    sx1262_clear_irq(SX_IRQ_ALL);

    ESP_LOGI(TAG, "configured %.3f MHz, SF%u, BW %.1f kHz, CR 4/%u, %d dBm", c->freq_mhz, c->sf,
             c->bw_khz, c->cr, pwr);
    return ESP_OK;
}

esp_err_t sx1262_start_rx(void)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    set_packet_params(255);
    sx1262_clear_irq(SX_IRQ_ALL);
    uint8_t p[3] = {0xff, 0xff, 0xff}; /* continuous */
    return cmd(OP_SET_RX, p, 3);
}

esp_err_t sx1262_transmit(const uint8_t *data, size_t len)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > 255) return ESP_ERR_INVALID_SIZE;
    set_standby();
    set_packet_params((uint8_t)len);
    s_txbuf[0] = OP_WRITE_BUF;
    s_txbuf[1] = 0x00;
    memcpy(&s_txbuf[2], data, len);
    esp_err_t err = xfer(2 + len);
    if (err != ESP_OK) return err;
    sx1262_clear_irq(SX_IRQ_ALL);
    uint8_t p[3] = {0, 0, 0}; /* no timeout; the mesh task has its own watchdog */
    return cmd(OP_SET_TX, p, 3);
}

esp_err_t sx1262_standby(void)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    return set_standby();
}

esp_err_t sx1262_sleep(void)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    uint8_t p = 0x04; /* warm start, keep configuration */
    esp_err_t err = cmd(OP_SET_SLEEP, &p, 1);
    if (err == ESP_OK) s_asleep = true;
    return err;
}

uint16_t sx1262_get_irq(void)
{
    uint8_t v[2] = {0};
    if (cmd_read(OP_GET_IRQ, NULL, 0, v, 2) != ESP_OK) return 0;
    return (uint16_t)((v[0] << 8) | v[1]);
}

void sx1262_clear_irq(uint16_t mask)
{
    uint8_t p[2] = {mask >> 8, mask & 0xff};
    cmd(OP_CLEAR_IRQ, p, 2);
}

esp_err_t sx1262_read_packet(uint8_t *buf, size_t cap, size_t *len, int16_t *rssi, float *snr)
{
    uint8_t st[2] = {0};
    esp_err_t err = cmd_read(OP_GET_RX_BUF_STATUS, NULL, 0, st, 2);
    if (err != ESP_OK) return err;
    size_t n = st[0];
    uint8_t off = st[1];
    if (n > cap) n = cap;
    /* ReadBuffer: op, offset, status(NOP), data... */
    memset(s_txbuf, 0, 3 + n);
    s_txbuf[0] = OP_READ_BUF;
    s_txbuf[1] = off;
    err = xfer(3 + n);
    if (err != ESP_OK) return err;
    memcpy(buf, &s_rxbuf[3], n);
    *len = n;

    uint8_t ps[3] = {0};
    cmd_read(OP_GET_PKT_STATUS, NULL, 0, ps, 3);
    *rssi = -(int16_t)ps[0] / 2;
    *snr = (int8_t)ps[1] / 4.0f;
    return ESP_OK;
}
