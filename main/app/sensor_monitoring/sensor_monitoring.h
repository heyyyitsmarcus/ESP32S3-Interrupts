#ifndef __SENSOR_MONITORING_H__
#define __SENSOR_MONITORING_H__

#include <stdbool.h>

#include "driver/pms5003/pms5003.h"
#include "driver/bme280/bme280_defs.h"
#include "driver/lora/lora.h"


// Environmental readings from each sensor
typedef struct
{
    struct bme280_data      bme;
    pms5003_measurement_t   pms;
} environmental_reading_t;


// Handles to set of sensor devices
typedef struct
{
    struct bme280_dev*  bme_handle;
    pms5003_config_t*   pms_handle;
    lora_config_t*      lora_handle;
} sensor_handles_t;


// Collect sensor data and initialize the environmental_reading_t struct.
//
// sensors : struct containing pointers to device instances for use in measurement functions
//
// reading : struct to hold data from each sensor
void collect_sensor_data( sensor_handles_t* sensors, environmental_reading_t* reading );


// Validate that the mac address in the received packet matches the sensor node.
//
// mac                      : mac address of sensor node.
//
// received_packet          : packet recieved through lora.
//
// received_packet_length   : lenght of packet.
//
// return                   : mac validity.
bool validate_mac(uint8_t* mac, uint8_t* received_packet, int received_packet_length);

#endif
