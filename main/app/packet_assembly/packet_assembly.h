#ifndef PACKET_ASSEMBLY_H
#define PACKET_ASSEMBLY_H

#include <stdint.h>
#include "app/sensor_monitoring/sensor_monitoring.h"

void prepare_packet(const uint32_t mac, const environmental_reading_t* reading, uint8_t* packet);

#endif
