#ifndef __PMS5003_H__
#define __PMS5003_H__

#include "driver/gpio.h"
#include "driver/uart.h"

typedef struct
{
    gpio_num_t set_pin;
    gpio_num_t reset_pin;
    gpio_num_t rxd_pin;
    gpio_num_t txd_pin;
    uart_port_t uart_controller;
} pms5003_config_t;

typedef struct
{
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
} pms5003_measurement_t;

void pms5003_setup(pms5003_config_t *inst);
void pms5003_make_measurement(pms5003_config_t *inst, pms5003_measurement_t *reading);
int pms5003_process_data(int len, uint8_t *data, pms5003_measurement_t *reading);
void pms5003_normal_mode(pms5003_config_t *inst);
void pms5003_sleep_mode(pms5003_config_t *inst);
void pms5003_reset(pms5003_config_t *inst);
void pms5003_print_measurement(pms5003_measurement_t *reading);

#endif
