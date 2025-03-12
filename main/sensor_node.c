#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "driver/bme280/bme_task.h"
#include "app/communication/communication.h"
#include "globals/globals.h"

void app_main(void)
{
    // Configure Lora.
    lora_config(&sensor_node_lora, NORMAL_FREQUENCY, LORA_CODING_RATE, LORA_BANDWIDTH, LORA_SPREAD_FACTOR);

    // Initialize sensors.
    init_bme(&bme);
    pms5003_setup(&pms0);
    
    // Create/start communication task.
    xTaskCreate(&communication, "communication", 1024 * 5, NULL, 6, NULL);
}
