#ifndef __LORA_H__
#define __LORA_H__

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "driver/gpio.h"


typedef struct {
    gpio_num_t miso_gpio;
    gpio_num_t sck_gpio;
    gpio_num_t mosi_gpio;
    gpio_num_t cs_gpio;
    gpio_num_t rst_gpio;
    spi_device_handle_t device_handle;
    spi_host_device_t host_id;
} lora_config_t;

int lora_read_reg(int reg, lora_config_t* lora_config);

void lora_config(lora_config_t* lora_config, long initial_freq, int coding_rate,
                 int bandwidth, int spread_factor);

void lora_reset(lora_config_t* lora_config);

void lora_explicit_header_mode(lora_config_t* lora_conifg);

void lora_implicit_header_mode(int size, lora_config_t* lora_conifg);

void lora_standby(lora_config_t* lora_conifg);

void lora_sleep(lora_config_t* lora_conifg);

void lora_receive_continuous(lora_config_t* lora_config);

void lora_set_tx_power(int level, lora_config_t* lora_conifg);

void lora_set_frequency(long frequency, lora_config_t* lora_conifg);

void lora_set_spreading_factor(int sf, lora_config_t* lora_conifg);

int lora_get_spreading_factor(lora_config_t* lora_conifg);

void lora_set_dio_mapping(int dio, int mode, lora_config_t* lora_conifg);

int lora_get_dio_mapping(int dio, lora_config_t* lora_conifg);

void lora_set_bandwidth(int sbw, lora_config_t* lora_conifg);

int lora_get_bandwidth(lora_config_t* lora_conifg);

void lora_set_coding_rate(int denominator, lora_config_t* lora_conifg);

int lora_get_coding_rate(lora_config_t* lora_conifg);

void lora_set_preamble_length(long length, lora_config_t* lora_conifg);

long lora_get_preamble_length(lora_config_t* lora_conifg);

void lora_set_sync_word(int sw, lora_config_t* lora_conifg);

void lora_enable_crc(lora_config_t* lora_conifg);

void lora_disable_crc(lora_config_t* lora_conifg);

int lora_init(lora_config_t* lora_conifg);

void lora_send_packet(uint8_t *buf, int size, lora_config_t* lora_conifg);

int lora_receive_packet(uint8_t *buf, int size, lora_config_t* lora_conifg);

int lora_received(lora_config_t* lora_conifg);

int lora_get_irq(lora_config_t* lora_conifg);

int lora_packet_rssi(lora_config_t* lora_conifg);

float lora_packet_snr(lora_config_t* lora_conifg);

void lora_close(lora_config_t* lora_conifg);

void lora_dump_registers(lora_config_t* lora_conifg);

#endif
