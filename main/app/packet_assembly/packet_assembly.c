#include "packet_assembly.h"

#include "app/sensor_monitoring/sensor_monitoring.h"


#define UINT32_MASK_MSB  0xFF000000
#define UINT32_MASK_MID1 0x00FF0000
#define UINT32_MASK_MID2 0x0000FF00
#define UINT32_MASK_LSB  0x000000FF

#define BYTE_SHIFT 8


static const uint32_t masks [ sizeof( uint32_t ) ] = { UINT32_MASK_MSB, UINT32_MASK_MID1, UINT32_MASK_MID2, UINT32_MASK_LSB };
static const uint32_t shifts[ sizeof( uint32_t ) ] = { BYTE_SHIFT * 3, BYTE_SHIFT * 2, BYTE_SHIFT, 0 };


static uint8_t* split_32( const uint32_t value, uint8_t* index );


void prepare_packet(const uint32_t mac, const environmental_reading_t* reading, uint8_t* packet)
{
    packet = split_32( mac, packet );

    packet = split_32( ( uint32_t )reading->bme.temperature , packet );
    packet = split_32( ( uint32_t )reading->bme.pressure    , packet );
    packet = split_32( ( uint32_t )reading->bme.humidity    , packet );

    packet = split_32( ( uint32_t )reading->pms.pm1_0_std   , packet );
    packet = split_32( ( uint32_t )reading->pms.pm2_5_std   , packet );
    packet = split_32( ( uint32_t )reading->pms.pm10_std    , packet );
    packet = split_32( ( uint32_t )reading->pms.pm1_0_atm   , packet );
    packet = split_32( ( uint32_t )reading->pms.pm2_5_atm   , packet );
    packet = split_32( ( uint32_t )reading->pms.pm10_atm    , packet );
}

static uint8_t* split_32( const uint32_t value, uint8_t* index )
{
    for ( uint8_t i = 0; i < sizeof( uint32_t ); i++, index++ )
    {
        *index = ( value & masks[ i ] ) >> shifts[ i ];
    }
    return index;
}

