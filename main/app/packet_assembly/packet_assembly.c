#include "packet_assembly.h"

#include <string.h>

#include "app/device/device.h"

#include "node_config.h"


void prepare_packet(const uint8_t* mac, const environmental_reading_t* reading, uint8_t* packet)
{
    memcpy( packet, mac, MAC_SIZE );
    packet += MAC_SIZE;

    memcpy( packet, &reading->bme.temperature, sizeof( double ) );
    packet += sizeof( double );
    memcpy( packet, &reading->bme.pressure   , sizeof( double ) );
    packet += sizeof( double );
    memcpy( packet, &reading->bme.humidity   , sizeof( double ) );
    packet += sizeof( double );

    memcpy( packet, &reading->pms.pm1_0_std  , sizeof( uint16_t ) );
    packet+= sizeof( uint16_t );
    memcpy( packet, &reading->pms.pm2_5_std  , sizeof( uint16_t ) );
    packet+= sizeof( uint16_t );
    memcpy( packet, &reading->pms.pm10_std   , sizeof( uint16_t ) );
    packet+= sizeof( uint16_t );
    memcpy( packet, &reading->pms.pm1_0_atm  , sizeof( uint16_t ) );
    packet+= sizeof( uint16_t );
    memcpy( packet, &reading->pms.pm2_5_atm  , sizeof( uint16_t ) );
    packet+= sizeof( uint16_t );
    memcpy( packet, &reading->pms.pm10_atm   , sizeof( uint16_t ) );
}

