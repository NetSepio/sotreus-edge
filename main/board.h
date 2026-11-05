// Heltec WiFi LoRa 32 V3 pin map (ESP32-S3FN8 + SX1262 + SSD1306).
#pragma once

#include "driver/gpio.h"

#define EDGE_HW_MODEL           "heltec_wifi_lora_32_v3"

// User button ("PRG"), active low, external pull-up.
#define PIN_BUTTON              GPIO_NUM_0
// White LED.
#define PIN_LED                 GPIO_NUM_35
// Vext rail switch: LOW powers the OLED (and the Vext header).
#define PIN_VEXT                GPIO_NUM_36

// OLED, I2C.
#define PIN_OLED_SDA            GPIO_NUM_17
#define PIN_OLED_SCL            GPIO_NUM_18
#define PIN_OLED_RST            GPIO_NUM_21
#define OLED_I2C_ADDR           0x3C

// Battery measurement: divider on GPIO1 (ADC1 ch0), gated by GPIO37.
#define PIN_VBAT_ADC            GPIO_NUM_1
#define PIN_VBAT_CTRL           GPIO_NUM_37

// SX1262 LoRa radio, SPI2.
#define PIN_LORA_NSS            GPIO_NUM_8
#define PIN_LORA_SCK            GPIO_NUM_9
#define PIN_LORA_MOSI           GPIO_NUM_10
#define PIN_LORA_MISO           GPIO_NUM_11
#define PIN_LORA_RST            GPIO_NUM_12
#define PIN_LORA_BUSY           GPIO_NUM_13
#define PIN_LORA_DIO1           GPIO_NUM_14
