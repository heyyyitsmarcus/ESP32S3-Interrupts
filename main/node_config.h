#ifndef __NODE_CONFIG_H__
#define __NODE_CONFIG_H__

// Communication macros.
#define MAC_SIZE                6
#define PACKET_SIZE             36 + MAC_SIZE
#define SLEEP_PING_SIZE         4

// SPI configuration for LoRa.
#define LORA_RST_GPIO    CONFIG_LORA_RST_GPIO
#define LORA_CS_GPIO     CONFIG_LORA_CS_GPIO
#define LORA_MOSI_GPIO   CONFIG_LORA_MOSI_GPIO
#define LORA_MISO_GPIO   CONFIG_LORA_MISO_GPIO
#define LORA_SCK_GPIO    CONFIG_LORA_SCK_GPIO
#define LORA_HOST_ID     SPI2_HOST
#define LORA_IRQ_GPIO    CONFIG_LORA_IRQ_GPIO

// LoRa configurations.
#define LORA_CODING_RATE        5
#define LORA_BANDWIDTH          7
#define LORA_SPREAD_FACTOR      7
#define NORMAL_FREQUENCY        915E6

// UART configuration for PMS5003
#define PMS_RST_GPIO     CONFIG_PMS_RST_GPIO
#define PMS_RXD_GPIO     CONFIG_PMS_RXD_GPIO
#define PMS_TXD_GPIO     CONFIG_PMS_TXD_GPIO
#define PMS_SET_GPIO     CONFIG_PMS_SET_GPIO
#define PMS_UART_NUM     UART_NUM_1

// I2C configuration for BME280
#define BME_SDA_GPIO     CONFIG_BME_SDA_GPIO
#define BME_SCL_GPIO     CONFIG_BME_SCL_GPIO
#define BME_I2C_PORT     I2C_NUM_0
#define BME_FREQUENCY    100E3

#endif

