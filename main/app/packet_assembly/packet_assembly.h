#ifndef PACKET_ASSEMBLY_H
#define PACKET_ASSEMBLY_H

#include <stdint.h>
#include "app/device/device.h"

// construct data packet consisting of device MAC address followed by sensor readings
//
// mac      : 6 byte device MAC address
//
// reading  : structure containing readings to place into packet
//
// packet   : buffer array used as the data packet
void prepare_packet(const uint8_t* mac, const environmental_reading_t* reading, uint8_t* packet);

#endif
