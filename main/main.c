#include <stdint.h>
#include <string.h>
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "esp_mac.h"
#include "pms.h"
#include "lora.h"
#include "bme.h"

#define MAC_SIZE 6
#define PACKET_SIZE 28 + MAC_SIZE

union lora_packet_u
{
    struct __attribute__ ((packed))
    {
        uint8_t mac[MAC_SIZE];
        pms5003_measurement_t pms_reading;
        bme680_measurement_t bme_reading;
    } reading;
    uint8_t raw[PACKET_SIZE];
};

static const char* TAG = "MAIN";

static uint8_t mac_address[MAC_SIZE] = { 0 };
static uint8_t lora_request_packet[PACKET_SIZE] = { 0 };
static union lora_packet_u lora_sensor_data_packet;
static pms5003_measurement_t pms_reading;
static bme680_measurement_t bme_reading;


static void collect_sensor_data ();
static bool validate_mac (uint8_t* received_packet, uint32_t received_packet_length);
static void prepare_packet (const bme680_measurement_t* bme_reading, const pms5003_measurement_t* pms_reading,
                            union lora_packet_u* lora_sensor_data_packet);


void app_main (void)
{
    esp_efuse_mac_get_default (mac_address);
    ESP_LOGI (TAG, "Node MAC: %02x%02x%02x%02x%02x%02x\n", mac_address[0], mac_address[1], mac_address[2],
              mac_address[3], mac_address[4], mac_address[5]);
    
    lora_init ();
    pms5003_init ();
    //bme680_init ();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // Ensure peripherals stay on during sleep

    while (true)
    {
        lora_receive();

        // Block until a packet is received or timeout
        if (lora_wait_for_packet_tick(portMAX_DELAY))
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            uint32_t received_packet_size = lora_receive_packet(lora_request_packet, sizeof(lora_request_packet));
            ESP_LOGI(TAG, "Packet received (%lu bytes)", received_packet_size);
            // To use validate function, uncomment the below code and ensure the sender is addressing the packet correctly with the right mac address
            if (received_packet_size != 0 ) // && validate_mac(lora_request_packet, received_packet_size))
            {
                ESP_LOGI(TAG, "Valid packet received (%lu bytes)", received_packet_size);
                collect_sensor_data();
                prepare_packet(&bme_reading, &pms_reading, &lora_sensor_data_packet);
                lora_send_packet(lora_sensor_data_packet.raw, sizeof(lora_sensor_data_packet.raw));
            }
        }
        else
        {
            ESP_LOGW(TAG, "Timed out waiting for LoRa packet");
        }
    }
}

// attempt with sleep implementation
// void app_main (void)
// {
//     // 1. Basic initialization
//     esp_efuse_mac_get_default (mac_address);
//     ESP_LOGI (TAG, "Node MAC: %02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1], mac_address[2],
//               mac_address[3], mac_address[4], mac_address[5]);
    
//     lora_init();
    
//     while (true)
//     {
//         // 2. Put the LoRa chip into receive mode
//         lora_receive();
//         //ESP_LOGI(TAG, "Waiting for interrupt...");

//         // 3. Wait for the LoRa interrupt to give the semaphore
//         if (lora_wait_for_packet_tick())
//         {
//             // 4. IMPORTANT: Do nothing else. Just log a simple message.
//             // Do not call lora_receive_packet() or any other function.
//             ESP_LOGI(TAG, "Success! Woke from interrupt without crashing.");

//             // 5. Delay to prevent spamming the log
//             vTaskDelay(pdMS_TO_TICKS(2000));
//         }
//     }
// }

static void collect_sensor_data ()
{
    //bme680_make_measurement (&bme_reading);
    pms5003_make_measurement (&pms_reading);
    printf ("%d, %d, %d, %d, %d, %d, %ld, %ld, %ld, %ld\n", pms_reading.pm1_0_std, pms_reading.pm2_5_std,
            pms_reading.pm10_std, pms_reading.pm1_0_atm, pms_reading.pm2_5_atm, pms_reading.pm10_atm,
            bme_reading.temp_comp, bme_reading.pres_comp, bme_reading.humd_comp, bme_reading.gas_comp);
    vTaskDelay (1000 / portTICK_PERIOD_MS);
}


static void prepare_packet (const bme680_measurement_t* bme_reading, const pms5003_measurement_t* pms_reading,
                            union lora_packet_u* lora_sensor_data_packet)
{
    memcpy(lora_sensor_data_packet->reading.mac, mac_address, MAC_SIZE);
    lora_sensor_data_packet->reading.pms_reading = *pms_reading;
    lora_sensor_data_packet->reading.bme_reading = *bme_reading;
}


static bool validate_mac (uint8_t* packet, uint32_t packet_length)
{
    if (packet_length < MAC_SIZE)
        return 0;
    bool is_valid_mac = true;
    for (uint8_t i = 0; i < MAC_SIZE; i++)
    {
        if (packet[i] != mac_address[i])
        {
            is_valid_mac = false;
            break;
        }
    }
    return is_valid_mac;
}
