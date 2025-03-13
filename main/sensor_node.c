#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "driver/bme280/bme280_i2c.h"
#include "driver/bme280/bme280.h"
#include "hal/i2c_types.h"

#include "app/sensor_monitoring/sensor_monitoring.h"
#include "app/communication/communication.h"
#include "node_config.h"

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
static const uint8_t bme280_i2c_port = I2C_NUM_0;

static struct bme280_dev bme280 =
{
    .intf = BME280_I2C_INTF,
    .read = bme280_i2c_read,
    .write = bme280_i2c_write,
    .delay_us = bme280_delay_us,
    .intf_ptr = ( void* )&bme280_i2c_port
};

static sensor_handles_t sensors =
{
    .bme_handle = &bme280,
    .pms_handle = &pms0,
    .lora_handle = &sensor_node_lora
};

void app_main(void)
{
    // Configure Lora
    lora_config(&sensor_node_lora, NORMAL_FREQUENCY, LORA_CODING_RATE, LORA_BANDWIDTH, LORA_SPREAD_FACTOR);

    // Initialize bme
    int8_t res = BME280_OK;

    res = bme280_init( &bme280 );
    if ( res == BME280_OK )
        printf("init GOOD\n");


    struct bme280_settings sett =
    {
        .osr_h = 0x1,
        .osr_p = 0x1,
        .osr_t = 0x1,
        .standby_time = 0,
        .filter = 0
    };

    res = bme280_set_sensor_settings( BME280_SEL_OSR_HUM | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP,
                                      &sett, &bme280 );
    if ( res == BME280_OK )
        printf("set settings GOOD\n");

    res = bme280_set_sensor_mode( BME280_POWERMODE_NORMAL, &bme280 );
    if ( res == BME280_OK )
        printf("set mode GOOD\n");


    // Initialize pms
    pms5003_setup(&pms0);
    
    // Create/start communication task.
    xTaskCreate(&communication, "communication", 1024 * 5, ( void* )&sensors, 6, NULL);
}
