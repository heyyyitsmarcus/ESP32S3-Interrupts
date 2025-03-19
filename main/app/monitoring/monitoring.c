#include "monitoring.h"

#include <stdbool.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app/device/device.h"

#include "app/packet_assembly/packet_assembly.h"

#include "driver/bme280/bme280.h"
#include "driver/bme280/bme280_defs.h"
#include "driver/pms5003/pms5003.h"
#include "driver/lora/lora.h"

#include "node_config.h"


// static binary semaphore used to notify monitoring
// task when to run
static SemaphoreHandle_t sema_handle = NULL;
static StaticSemaphore_t sema;


// Tag to identify module in logs
static const char* TAG = "monitoring";


// Buffer to hold incoming and outgoing lora packets
static uint8_t lora_buffer[ PACKET_SIZE ] = { 0 };


// Struct to hold readings from sensors
static environmental_reading_t sensor_readings = { 0 };


static bool validate_mac(uint8_t* mac, uint8_t* received_packet, uint32_t received_packet_length);


static void collect_sensor_data( device_handles_t* devices, environmental_reading_t* reading );


void semaphore_init()
{
    sema_handle = xSemaphoreCreateBinaryStatic( &sema );
}


void lora_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    // notify monitoring task of reception event by unlocking semaphore
    xSemaphoreGiveFromISR( sema_handle, &higher_priority_task_woken );
    portYIELD_FROM_ISR( higher_priority_task_woken );
}


// Main monitoring function
void monitoring()
{
    device_handles_t* devices = device_get_handles();

    uint8_t* mac = device_get_mac();

    while ( true )
    {
        // Block until lora isr unlocks semaphore signifying reception event
        xSemaphoreTake( sema_handle, portMAX_DELAY );

        uint32_t received_packet_size = lora_receive_packet( lora_buffer, sizeof( lora_buffer ), devices->lora_handle );

        if ( ( received_packet_size != 0 ) && validate_mac( mac, lora_buffer, received_packet_size ) )
        {
            ESP_LOGI( TAG, "%lu byte packet received.", received_packet_size );
            // Handle data request.
            ESP_LOGI( TAG, "Data request received." );

            collect_sensor_data( devices, &sensor_readings );

            prepare_packet( mac, &sensor_readings, lora_buffer );

            lora_send_packet( lora_buffer, sizeof( lora_buffer ), devices->lora_handle );
            ESP_LOGI( TAG, "Data sent to central. Returning to receive mode.\n" );
            lora_receive_continuous( devices->lora_handle );
        } 
    }
}


// Check if received packet contains nodes own MAC address
//
// mac                  : nodes MAC address
//
// received_packet      : incoming packet to check
//
// received_packet_len  : size in bytes of incoming packet
//
// return : boolean whether MAC matches self
static bool validate_mac(uint8_t* mac, uint8_t* received_packet, uint32_t received_packet_length)
{
    if (received_packet_length < MAC_SIZE) return 0;
    bool is_valid_mac = true;
    for (uint8_t i = 0; i < MAC_SIZE; i++)
    {
        if (received_packet[i] != mac[i])
        {
            is_valid_mac = false;
            break;
        }
    }
    return is_valid_mac;
}


// Poll all sensors
//
// devices : handles to devices
//
// reading : struct to hold read back readings
static void collect_sensor_data( device_handles_t* devices, environmental_reading_t* reading )
{
    pms5003_make_measurement( devices->pms_handle, &reading->pms );
    bme280_get_sensor_data( BME280_ALL, &reading->bme, devices->bme_handle );
}

