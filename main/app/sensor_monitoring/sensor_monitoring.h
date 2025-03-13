#ifndef SENSOR_MONITORING_H
#define SENSOR_MONITORING_H

#include "esp_log.h"

#include "driver/bme280/bme_task.h"
#include "driver/pms5003/pms5003.h"
#include "driver/lora/lora.h"

#include "app/packet_assembly/packet_assembly.h"
#include "globals/globals.h"


// Collect sensor data from queues and initialize the environmental_reading_t struct.
//
// @param bme_data : struct to hold the bme data coming in from the queue.
//
// @param pms_data : struct to hold the pms data coming in from the queue.
//
// @param mq7_data : struct to hold the mq7 data coming in from the queue.
//
// @param sensor_readings : struct to consolidate each of the sensor datas
// received from the queue.
void collect_sensor_data(bme_measurement_t* bme_data,
                         pms5003_measurement_t* pms_data,
                         environmental_reading_t* sensor_readings);


// Initialize a environmental_reading_t struct from sensor data
//
// @param sensorReadings : empty environmental_reading_t struct to be initalized.
//
// @param bmeReading : bme sensor data.
//
// @param pmsReading : pms sensor data.
//
// @param mq7Reading : mq7 sensor data.
void initReading(environmental_reading_t* sensorReadings, const bme_measurement_t* bmeReading,
                 const pms5003_measurement_t* pmsReading);


// Validate that the mac address in the received packet matches the sensor node.
//
// @param mac : mac address of sensor node.
//
// @param received_packet : packet recieved through lora.
//
// @param received_packet_length : lenght of packet.
//
// return : 1 for true, 0 for false.
//
int validate_mac(uint8_t* mac, uint8_t* received_packet, int received_packet_length);

#endif
