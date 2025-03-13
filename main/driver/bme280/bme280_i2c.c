#include "bme280_i2c.h"

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal/gpio_types.h"
#include "driver/i2c.h"

#include "assert.h"
#include "esp_err.h"

#include "bme280_defs.h"


#define WRITE_TRANSACTION_COUNT 4
#define ONE_MS_IN_US            1000


void bme280_i2c_config( const i2c_port_t port, const gpio_num_t sda_gpio, 
                        const gpio_num_t scl_gpio, const uint32_t clk_speed )
{
    esp_err_t res;
    i2c_config_t conf =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = clk_speed,
        .clk_flags = 0
    };
    res = i2c_param_config( port, &conf );
    ESP_ERROR_CHECK( res );

    res = i2c_driver_install( port, conf.mode, 0, 0, 0 );
    ESP_ERROR_CHECK( res );
}


int8_t bme280_i2c_read( uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr )
{
    esp_err_t res;
    // Take void pointer as desired i2c port 
    uint8_t* i2c_port = ( uint8_t* )( intf_ptr );

    res = i2c_master_write_read_device( *i2c_port, BME280_I2C_ADDR_SEC, &reg_addr, 1, reg_data, len, portMAX_DELAY );
    return res;
}


int8_t bme280_i2c_write( uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr )
{
    esp_err_t res;
    // Take void pointer as desired i2c port 
    uint8_t* i2c_port = ( uint8_t* )( intf_ptr );

    uint8_t buffer[ I2C_LINK_RECOMMENDED_SIZE( WRITE_TRANSACTION_COUNT ) ] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static( buffer, sizeof( buffer ) );

    assert( handle != NULL );

    res = i2c_master_start( handle );

    if ( res != ESP_OK )
        goto end;

    res = i2c_master_write_byte( handle, ( BME280_I2C_ADDR_SEC << 1 ) | I2C_MASTER_WRITE, true );
    if ( res != ESP_OK )
        goto end;

    res = i2c_master_write_byte( handle, reg_addr, true );
    if ( res != ESP_OK )
        goto end;

    res = i2c_master_write( handle, reg_data, len, true );
    if ( res != ESP_OK )
        goto end;

    i2c_master_stop( handle );
    res = i2c_master_cmd_begin( *i2c_port, handle, portMAX_DELAY );

end:
    i2c_cmd_link_delete_static( handle );
    return res;
}


// Convert microseconds to milliseconds
//
// period_us : period in microseconds
//
// return : calculated milliseconds
static uint32_t us_to_ms( uint32_t period_us )
{
    if ( period_us < ONE_MS_IN_US )
        return 1;
    else
        return period_us / ONE_MS_IN_US;
}


void bme280_delay_us( uint32_t period, void* intf_ptr )
{
    vTaskDelay( pdMS_TO_TICKS( us_to_ms( period ) ) );
}
