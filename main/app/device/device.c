#include "device.h"

#include "hal/i2c_types.h"
#include "esp_mac.h"

#include "driver/bme280/bme280.h"
#include "driver/bme280/bme280_defs.h"
#include "driver/bme280/bme280_i2c.h"
#include "driver/pms5003/pms5003.h"
#include "driver/lora/lora.h"

#include "node_config.h"


// Lora pin definitions.
static lora_config_t lora = 
{
    .miso_gpio  = LORA_MISO_GPIO,
    .sck_gpio   = LORA_SCK_GPIO,
    .mosi_gpio  = LORA_MOSI_GPIO,
    .cs_gpio    = LORA_CS_GPIO,
    .rst_gpio   = LORA_RST_GPIO,
    .host_id    = LORA_HOST_ID
};

// PMS pin definitions.
static pms5003_config_t pms = 
{
    .reset_pin          = PMS_RST_GPIO,
    .rxd_pin            = PMS_RXD_GPIO,
    .txd_pin            = PMS_TXD_GPIO,
    .set_pin            = PMS_SET_GPIO,
    .uart_controller    = PMS_UART_NUM
};


// bme i2c port
static const uint8_t bme280_i2c_port = BME_I2C_PORT;
// bme callback definitions
static struct bme280_dev bme =
{
    .intf       = BME280_I2C_INTF,
    .read       = bme280_i2c_read,
    .write      = bme280_i2c_write,
    .delay_us   = bme280_delay_us,
    .intf_ptr   = ( void* )&bme280_i2c_port
};


// device handles
static device_handles_t devices =
{
    .bme_handle     = &bme,
    .pms_handle     = &pms,
    .lora_handle    = &lora
};


// MAC of node
static uint8_t mac_address[ MAC_SIZE ] = { 0 };
// Bool to keep track if MAC has been initialized
static bool mac_initialized = false;


device_handles_t* device_get_handles()
{
    return &devices;
}


uint8_t* device_get_mac()
{
    if ( !mac_initialized )
        esp_efuse_mac_get_default( mac_address );
    return mac_address;
}


void device_init()
{
    // Configure Lora
    lora_config( &lora, NORMAL_FREQUENCY, LORA_CODING_RATE, LORA_BANDWIDTH, LORA_SPREAD_FACTOR );
    lora_receive_continuous( &lora );

    // Initialize bme
    bme280_i2c_config( bme280_i2c_port, BME_SDA_GPIO, BME_SCL_GPIO, BME_FREQUENCY );
    int8_t res = BME280_OK;

    res = bme280_init( &bme );
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
                                      &sett, &bme );
    if ( res == BME280_OK )
        printf("set settings GOOD\n");

    res = bme280_set_sensor_mode( BME280_POWERMODE_NORMAL, &bme );
    if ( res == BME280_OK )
        printf("set mode GOOD\n");

    // Initialize pms
    pms5003_setup( &pms );
    pms5003_normal_mode( &pms );
}

    
