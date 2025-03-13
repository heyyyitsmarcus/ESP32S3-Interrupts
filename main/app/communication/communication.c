#include "communication.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/lora/lora.h"

#include "app/packet_assembly/packet_assembly.h"
#include "app/sensor_monitoring/sensor_monitoring.h"

#include "node_config.h"


void communication( void* p_arg )
{
    sensor_handles_t* sensors = ( sensor_handles_t* )p_arg; 

    // Get mac address.
    uint8_t mac[ MAC_SIZE ] = { 0 };
    esp_efuse_mac_get_default( mac );
    printf( "%02x%02x%02x%02x%02x%02x\n", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ]) ;

    uint32_t mac32 = ( uint32_t )( ( mac[ 2 ] << 24 ) | ( mac[ 3 ] << 16 ) | ( mac[ 4 ] << 8 ) << mac[ 5 ] );

    environmental_reading_t sensor_readings = { 0 };

    uint8_t lora_buffer[ PACKET_SIZE ];

    UBaseType_t high_water_mark;
    
    lora_receive_continuous( sensors->lora_handle );
    while ( 1 )
    {
        // Returns the minimum amount of words remaining in the stack depth since
        // task creation.
        high_water_mark = uxTaskGetStackHighWaterMark( NULL );
        ESP_LOGD( pcTaskGetName( NULL ),
                  "Minimum stack space remaining since creation: %d words.", high_water_mark );

        vTaskDelay( pdMS_TO_TICKS( 100 ) );
        if ( lora_received( sensors->lora_handle ) )
        {
            int received_packet_size = lora_receive_packet( lora_buffer, sizeof( lora_buffer ), sensors->lora_handle );
            ESP_LOGI( pcTaskGetName( NULL ), "%d byte packet received.\n", received_packet_size );

            if ( validate_mac( mac, lora_buffer, received_packet_size ) )
            {
                // Handle data request.
                ESP_LOGI( pcTaskGetName( NULL ), "Data request received." );

                collect_sensor_data( sensors, &sensor_readings );

                prepare_packet( mac32, &sensor_readings, lora_buffer );

                lora_send_packet( lora_buffer, sizeof( lora_buffer ), sensors->lora_handle );
                ESP_LOGI( pcTaskGetName( NULL ), "Data sent to central. Returning to receive mode." );
                lora_receive_continuous( sensors->lora_handle );
            } 
        }
    }
}

