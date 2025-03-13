#ifndef __BME280_I2C_H__
#define __BME280_I2C_H__

#include <stdint.h>

#include "hal/gpio_types.h"
#include "hal/i2c_types.h"

// Configure I2C peripheral as master
//
// port      : i2c port for communication
//
// sda_gpio  : gpio number for sda pin
//
// scl_gpio  : gpio number for scl pin
//
// clk_speed : i2c clock speed
void bme280_i2c_config( const i2c_port_t port, const gpio_num_t sda_gpio, 
                        const gpio_num_t scl_gpio, const uint32_t clk_speed );

// Callback for BME280 API read function (see BME280 driver for more details)
int8_t bme280_i2c_read( uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr );

// Callback for BME280 API write function (see BME280 driver for more details)
int8_t bme280_i2c_write( uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr );

// Callback for BME280 API delay function (see BME280 driver for more details)
void bme280_delay_us( uint32_t period, void* intf_ptr );

#endif
