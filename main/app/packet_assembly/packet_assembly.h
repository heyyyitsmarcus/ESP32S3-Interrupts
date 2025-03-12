#ifndef PACKET_ASSEMBLY_H
#define PACKET_ASSEMBLY_H

#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


// Struct to hold environmental data from sensors.
typedef struct {
    // BME
    int16_t temperature;
    int32_t pressure;
    int16_t humidity;

    // PMS
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
} environmental_reading_t;


// Print out the values of a environmental_reading_t struct.
//
// @param reading : environmental_reading_t struct that you would like to print its data.
void printReading(const environmental_reading_t* reading);


// Populate buffer with MAC address and sensor readings.
//
// @param mac : MAC address of esp32.
// 
// @param reading : struct that holds the sensor data.
//
// @param buf : buffer to hold the data to be sent via
// LoRa. Should hold exactly 42 values: buf[42].
void prepareBuffer(uint8_t* mac, const environmental_reading_t* reading, uint8_t* buf);


// Extract data from buffer into a environmental_reading_t struct.
//
// @param buf: buffer holding the broken down sensor readings that 
// you want to extract.
//
// @param reading : empty environmental_reading_t struct to store the extracted data
// from the buffer.
void extractBuffer(uint8_t* buf, environmental_reading_t* reading);


// Insert the MAC address to the first 6 bytes of the buffer to be sent.
//
// @param buf : lora buffer.
//
// @param mac : MAC address of esp32.
uint8_t* insertMac(uint8_t* buf, uint8_t* mac);


// Break down an int16_t value into two uint8_t values and store them in an array.
//
// @param value : int16_t value to be broken down.
//
// @param startingIndex : the address of the array index from where to start storing 
// the broken down values.
uint8_t* sixteenBitMaskShift(const int16_t value, uint8_t* startingIndex);


// Break down an int32_t value into four uint8_t values and store them in an array.
//
// @param value : int32_t value to be broken down.
//
// @param startingIndex : the address of the array index from where to start storing 
// the broken down values.
uint8_t* thirtyTwoBitMaskShift(const int32_t value, uint8_t* startingIndex);


// Concatinate uint8_t values from an array into a int16_t value.
//
// @param startingIndex : the address of a pointer pointing to the index from where 
// you would like to start concatinating the uint8_t values.
int16_t sixteenBitConcatenated(uint8_t** startingIndex);


// Concatinate uint8_t values from an array into a int32_t value.
//
// @param startingIndex : the address of a pointer pointing to the index from where 
// you would like to start concatinating the uint8_t values.
int32_t thirtyTwoBitConcatenated(uint8_t** startingIndex);

#endif
