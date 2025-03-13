#include "sensor_monitoring.h"

#include "driver/pms5003/pms5003.h"
#include "driver/bme280/bme280.h"
#include "driver/bme280/bme280_defs.h"

#include "node_config.h"


void collect_sensor_data( sensors_handles_t* sensors, environmental_reading_t* reading )
{
    pms5003_make_measurement( sensors->pms_handle, &reading->pms );
    bme280_get_sensor_data( BME280_ALL, &reading->bme, sensors->bme_handle );
}


int validate_mac(uint8_t* mac, uint8_t* received_packet, int received_packet_length)
{
    if (received_packet_length < MAC_SIZE) return 0;
    int is_valid_mac = 1;
    for (int i = 0; i < MAC_SIZE; i++)
    {
        if (received_packet[i] != mac[i])
        {
            is_valid_mac = 0;
            break;
        }
    }
    return is_valid_mac;
}

