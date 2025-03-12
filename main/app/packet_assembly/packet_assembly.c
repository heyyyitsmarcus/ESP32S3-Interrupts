#include "packet_assembly.h"

#define INT16_MASK1 0xFF00
#define INT16_MASK2 0x00FF

#define INT32_MASK1 0xFF000000
#define INT32_MASK2 0x00FF0000
#define INT32_MASK3 0x0000FF00
#define INT32_MASK4 0x000000FF

#define EIGHT_BIT_SHIFT 8


void printReading(const environmental_reading_t* reading)
{
    ESP_LOGI(pcTaskGetName(NULL), "Temperature: %d", reading->temperature);
    ESP_LOGI(pcTaskGetName(NULL), "Pressure: %ld", reading->pressure);
    ESP_LOGI(pcTaskGetName(NULL), "Humidity: %d", reading->humidity);
    
    ESP_LOGI(pcTaskGetName(NULL), "PM 1.0 std: %d", reading->pm1_0_std);
    ESP_LOGI(pcTaskGetName(NULL), "PM 2.5 std: %d", reading->pm2_5_std);
    ESP_LOGI(pcTaskGetName(NULL), "PM 10 std: %d", reading->pm10_std);
    ESP_LOGI(pcTaskGetName(NULL), "PM 1.0 atm: %d", reading->pm1_0_atm);
    ESP_LOGI(pcTaskGetName(NULL), "PM 2.5 atm: %d", reading->pm2_5_atm);
    ESP_LOGI(pcTaskGetName(NULL), "PM 10 atm: %d", reading->pm10_atm);
}


void prepareBuffer(uint8_t* mac, const environmental_reading_t* reading, uint8_t* buf)
{
    buf = insertMac(buf, mac);

    buf = sixteenBitMaskShift(reading->temperature, buf);
    buf = thirtyTwoBitMaskShift(reading->pressure, buf);
    buf = sixteenBitMaskShift(reading->humidity, buf);

    buf = sixteenBitMaskShift(reading->pm1_0_std, buf);
    buf = sixteenBitMaskShift(reading->pm2_5_std, buf);
    buf = sixteenBitMaskShift(reading->pm10_std, buf);
    buf = sixteenBitMaskShift(reading->pm1_0_atm, buf);
    buf = sixteenBitMaskShift(reading->pm2_5_atm, buf);
    buf = sixteenBitMaskShift(reading->pm10_atm, buf);
}


void extractBuffer(uint8_t* buf, environmental_reading_t* reading)
{
    buf += 6;
    
    reading->temperature = sixteenBitConcatenated(&buf);
    reading->pressure = thirtyTwoBitConcatenated(&buf);
    reading->humidity = sixteenBitConcatenated(&buf);

    reading->pm1_0_std = sixteenBitConcatenated(&buf);
    reading->pm2_5_std = sixteenBitConcatenated(&buf);
    reading->pm10_std = sixteenBitConcatenated(&buf);
    reading->pm1_0_atm = sixteenBitConcatenated(&buf);
    reading->pm2_5_atm = sixteenBitConcatenated(&buf);
    reading->pm10_atm = sixteenBitConcatenated(&buf);
}


uint8_t* insertMac(uint8_t* buf, uint8_t* mac)
{
    for (int i = 0; i < 6; i++)
    {
        *buf = *mac;
        buf++;
        mac++;
    }
    return buf;
}


uint8_t* sixteenBitMaskShift(const int16_t value, uint8_t* startingIndex)
{
	uint16_t masked1 = value & INT16_MASK1;
	uint8_t shifted1 = masked1>>(EIGHT_BIT_SHIFT);

	uint16_t masked2 = value & INT16_MASK2;
    uint8_t shifted2 = masked2;
    
    uint8_t tempArr[] = {shifted1, shifted2};

    for (int i = 0; i < 2; i++)
    {
        *startingIndex = tempArr[i];
        startingIndex++;
    }

    return startingIndex;
}


uint8_t* thirtyTwoBitMaskShift(const int32_t value, uint8_t* startingIndex)
{
	uint32_t masked1 = value & INT32_MASK1;
	uint8_t shifted1 = masked1>>(EIGHT_BIT_SHIFT*3);

	uint32_t masked2 = value & INT32_MASK2;
	uint8_t shifted2 = masked2>>(EIGHT_BIT_SHIFT*2);

	uint32_t masked3 = value & INT32_MASK3;
	uint8_t shifted3 = masked3>>EIGHT_BIT_SHIFT;

	uint32_t masked4 = value & INT32_MASK4;
    uint8_t shifted4 = masked4;

    uint8_t tempArr[] = {shifted1, shifted2, shifted3, shifted4};

    for (int i = 0; i < 4; i++)
    {
        *startingIndex = tempArr[i];
        startingIndex++;
    }

    return startingIndex;
}


int16_t sixteenBitConcatenated(uint8_t** startingIndex)
{
    uint16_t firstPart = **startingIndex << EIGHT_BIT_SHIFT;
    (*startingIndex)++;

    uint16_t secondPart = **startingIndex;
    (*startingIndex)++; // Iterator is now where the following piece of data would start.

	int16_t concatinated = firstPart | secondPart;

	return concatinated;
}


int32_t thirtyTwoBitConcatenated(uint8_t** startingIndex) 
{
    uint32_t firstPart = **startingIndex << (EIGHT_BIT_SHIFT * 3);
    (*startingIndex)++;

    uint32_t secondPart = **startingIndex << (EIGHT_BIT_SHIFT * 2);
    (*startingIndex)++;

    uint32_t thirdPart = **startingIndex << (EIGHT_BIT_SHIFT);
    (*startingIndex)++;

    uint32_t fourthPart = **startingIndex;
    (*startingIndex)++; // Iterator is now where the following piece of data would start.

    int32_t concatinated = firstPart | secondPart | thirdPart | fourthPart;

	return concatinated;
}

