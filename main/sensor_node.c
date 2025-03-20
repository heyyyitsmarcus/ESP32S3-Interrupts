#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/device/device.h"
#include "app/monitoring/monitoring.h"

#include "esp_log.h"

#define STACK_SIZE 1024 * 2

static const char* TAG = "main";

StaticTask_t monitoring_tcb;
StackType_t  monitoring_stack[ STACK_SIZE ];

void app_main(void)
{
    semaphore_init();

    int8_t res = device_init();
    if ( res == 0 )
        ESP_LOGI( TAG, "Device initialization success.");


    uint8_t* mac = device_get_mac();
    ESP_LOGI( TAG, "Node MAC: %02x%02x%02x%02x%02x%02x\n", mac[ 0 ], mac[ 1 ], 
                                                           mac[ 2 ], mac[ 3 ], 
                                                           mac[ 4 ], mac[ 5 ]) ;

    xTaskCreateStaticPinnedToCore( monitoring, "Monitoring", STACK_SIZE, NULL, 2, 
                                   monitoring_stack, &monitoring_tcb, 0 );
}
