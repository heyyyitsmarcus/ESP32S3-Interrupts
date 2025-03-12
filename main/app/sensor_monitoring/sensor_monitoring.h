#ifndef SENSOR_MONITORING_H
#define SENSOR_MONITORING_H

#include "esp_log.h"

#include "driver/bme280/bme_task.h"
#include "driver/pms5003/pms5003.h"
#include "driver/lora/lora.h"

#include "app/packet_assembly/packet_assembly.h"
#include "globals/globals.h"


// Monitor environment for a set amount of seconds. If a fire is detected
// will return early and change its frequency to FIRE_FREQUENCY.
//
// @param bme_data : bme data from queue.
//
// @param pms_data : pms data from queue.
//
// @param mq7_data : mq7 data from queue.
//
// @param sensor_readings : struct to consolidate all readings into.
//
// @param seconds_to_delay : how long to collect data for.
//
// @param lora_config : specific lora to transmit through.
//
// return: 1 if a fire was detected, 0 if no fire was detected.
//
int sleep_montior(bme_measurement_t* bme_data, pms5003_measurement_t* pms_data,
                  environmental_reading_t* sensor_readings,
                  int seconds_to_delay, lora_config_t* lora_config);


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


// Check sensor data for indication of a fire.
//
// @param sensor_readings : readings from sensors.
//
// return: 1 for true, 0 for false.
//
int check_for_fire(environmental_reading_t* sensor_readings);


// Send a fire alert to central node.
//
// @param mac : MAC address of esp32.
//
// @param lora_config : specific lora to transmit through.
//
void send_fire_alert(uint8_t* mac, lora_config_t* lora_config);


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


// Check if a received packet is a sleep ping
//
// @param received_packet : packet recieved through lora.
//
// return : 1 for true, 0 for false.
//
int IsSleepPing(uint8_t* recieved_packet);

#endif
