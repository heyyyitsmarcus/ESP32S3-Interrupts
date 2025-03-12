#ifndef __BME_TASK_H__
#define __BME_TASK_H__

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "bme280.h"

#define SDA_PIN CONFIG_BME_SDA_GPIO
#define SCL_PIN CONFIG_BME_SCK_GPIO

#define TAG_BME280 "BME280"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

typedef struct
{
    int16_t temperature;
    int32_t pressure;
    int16_t humidity;
} bme_measurement_t;

void init_bme(struct bme280_t* bme);

void bme_make_measurement(bme_measurement_t* bme_measurement);

#endif
