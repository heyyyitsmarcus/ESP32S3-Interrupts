#include "communication.h"

void communication()
{
    // Get mac address.
    uint8_t mac[MAC_SIZE] = {0};
    esp_efuse_mac_get_default(mac);
    printf("%02x%02x%02x%02x%02x%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    bme_measurement_t bme_data = {0};
    pms5003_measurement_t pms_data = {0};
    environmental_reading_t sensor_readings = {0};
    uint8_t lora_buffer[PACKET_SIZE];

    UBaseType_t high_water_mark;
    
    // Send MAC address to central node.
    lora_send_packet(mac, sizeof(mac), &sensor_node_lora);
    ESP_LOGI(pcTaskGetName(NULL), "MAC address sent.");
    
    lora_receive_continuous(&sensor_node_lora);
    while (1)
    {
        // Returns the minimum amount of words remaining in the stack depth since
        // task creation.
        high_water_mark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(pcTaskGetName(NULL),
                 "Minimum stack space remaining since creation: %d words.", high_water_mark);

        vTaskDelay(pdMS_TO_TICKS(100));
        if (lora_received(&sensor_node_lora))
        {
            int received_packet_size = lora_receive_packet(lora_buffer, sizeof(lora_buffer), &sensor_node_lora);
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received.\n", received_packet_size);

            if (validate_mac(mac, lora_buffer, received_packet_size))
            {
                // Handle data request.
                ESP_LOGI(pcTaskGetName(NULL), "Data request received.");
                collect_sensor_data(&bme_data, &pms_data, &sensor_readings);
                printReading(&sensor_readings);
                prepareBuffer(mac, &sensor_readings, lora_buffer);

                //vTaskDelay(pdMS_TO_TICKS(500));

                lora_send_packet(lora_buffer, sizeof(lora_buffer), &sensor_node_lora);
                ESP_LOGI(pcTaskGetName(NULL), "Data sent to central. Returning to receive mode.");
                lora_receive_continuous(&sensor_node_lora);
            } 
        }
    }
}

