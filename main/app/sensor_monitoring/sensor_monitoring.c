#include "sensor_monitoring.h"

void collect_sensor_data(bme_measurement_t* bme_data, pms5003_measurement_t* pms_data, environmental_reading_t* sensor_readings)
{
    pms5003_make_measurement(&pms0, pms_data);
    bme_make_measurement(bme_data);
    initReading(sensor_readings, bme_data, pms_data);
}


void initReading(environmental_reading_t* sensorReadings, const bme_measurement_t* bmeReading,
                 const pms5003_measurement_t* pmsReading) 
{
    sensorReadings->temperature = bmeReading->temperature;
    sensorReadings->pressure = bmeReading->pressure;
    sensorReadings->humidity = bmeReading->humidity;

    sensorReadings->pm1_0_std = pmsReading->pm1_0_std;
    sensorReadings->pm2_5_std = pmsReading->pm2_5_std;
    sensorReadings->pm10_std = pmsReading->pm10_std;
    sensorReadings->pm1_0_atm = pmsReading->pm1_0_atm;
    sensorReadings->pm2_5_atm = pmsReading->pm2_5_atm;
    sensorReadings->pm10_atm = pmsReading->pm10_atm;
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

