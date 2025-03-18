#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/device/device.h"
#include "app/monitoring/monitoring.h"

#define STACK_SIZE 1024 * 2

StaticTask_t monitoring_task_buffer;
StackType_t  monitoring_stack[ STACK_SIZE ];

void app_main(void)
{
    device_init();

    xTaskCreateStatic( monitoring, "Monitoring", STACK_SIZE, NULL, 2, monitoring_stack, &monitoring_task_buffer );
}
