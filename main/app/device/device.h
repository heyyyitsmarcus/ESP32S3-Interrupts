#ifndef __DEVICE_H__
#define __DEVICE_H__


#include "driver/bme280/bme280_defs.h"
#include "driver/pms5003//pms5003.h"
#include "driver/lora/lora.h"


// Handles to set of sensor devices
typedef struct
{
    struct bme280_dev*  bme_handle;
    pms5003_config_t*   pms_handle;
    lora_config_t*      lora_handle;
} device_handles_t;


// Environmental readings from each sensor
typedef struct
{
    struct bme280_data      bme;
    pms5003_measurement_t   pms;
} environmental_reading_t;


// Get handles to device instances
//
// return : pointer to device instances
device_handles_t* device_get_handles();


// Get MAC address of MCU
uint8_t* device_get_mac();


// Initialize all devices (lora, bme, pms)
//
// return: 0 for success
int8_t device_init();


#endif
