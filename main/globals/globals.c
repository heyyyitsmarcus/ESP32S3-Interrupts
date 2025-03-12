#include "globals/globals.h"

// Lora pin definitions.
lora_config_t sensor_node_lora = {
    .miso_gpio = SENSOR_LORA_MISO_GPIO,
    .sck_gpio = SENSOR_LORA_SCK_GPIO,
    .mosi_gpio = SENSOR_LORA_MOSI_GPIO,
    .cs_gpio = SENSOR_LORA_CS_GPIO,
    .rst_gpio = SENSOR_LORA_RST_GPIO,
    .host_id = SENSOR_LORA_HOST_ID};

// PMS pin definitions.
pms5003_config_t pms0 = {
    .reset_pin = CONFIG_PMS_RST_GPIO,
    .rxd_pin = CONFIG_PMS_RXD_GPIO,
    .txd_pin = CONFIG_PMS_TXD_GPIO,
    .set_pin = CONFIG_PMS_SET_GPIO,
    .uart_controller = UART_NUM_1
};

struct bme280_t bme;
