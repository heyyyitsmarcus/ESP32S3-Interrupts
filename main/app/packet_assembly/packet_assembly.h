#ifndef PACKET_ASSEMBLY_H
#define PACKET_ASSEMBLY_H

#include <stdint.h>
#include "app/device/device.h"

void prepare_packet(const uint8_t* mac, const environmental_reading_t* reading, uint8_t* packet);

#endif
