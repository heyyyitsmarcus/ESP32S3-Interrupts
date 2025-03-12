#include "pms5003.h"

#include "esp_log.h"

#define BAUD_RATE             9600
#define DATA_PACKET_SIZE      32
#define START_CHARACTER_1     0x42
#define START_CHARACTER_2     0x4D
#define FRAME_LEN             28

#define RX_BUFFER_SIZE        160

#define CHECK_CODE_HI_INDEX 30
#define CHECK_CODE_LOW_INDEX  31

const char* TAG = "PMS5003 Driver";

void pms5003_setup(pms5003_config_t *inst)
{
    // Initializing UART interface
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(inst->uart_controller, &uart_config);
    uart_set_pin(inst->uart_controller, inst->txd_pin, inst->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Size of uart RX FIFO must be greater than the size of the hardware FIFO which is 128 bytes.
    uart_driver_install(inst->uart_controller, RX_BUFFER_SIZE, 0, 0, NULL, 0);

    // Initializing GPIO pins
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << inst->set_pin) | (1ULL << inst->reset_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    pms5003_normal_mode(inst);
    pms5003_reset(inst);
}

void pms5003_make_measurement(pms5003_config_t *inst, pms5003_measurement_t *reading)
{
    uint8_t data[RX_BUFFER_SIZE] = {0};
    int data_len = 0;
    
    // Get the length of the current data within uart controller RX FIFO buffer.
    uart_get_buffered_data_len(inst->uart_controller, (size_t*)&data_len);
    // Read in the returned length amount of bytes. This may be more than the data packet
    // size (32), one option is to read regardless and ignore the extra data or flush the
    // RX buffer when this happens. Choosing the first.
    data_len = uart_read_bytes(inst->uart_controller, (void*)data, data_len, 20);
    
    if (data_len > 0 )
        pms5003_process_data(data_len, data, reading);
}

int pms5003_process_data(int len, uint8_t *data, pms5003_measurement_t *reading)
{
    // Length test. Is the length of the uart data a length divisible by the data packet
    // size. Some uart reads will take in two or more data packets, we will choose to only
    // read the first.
    if (len % DATA_PACKET_SIZE != 0)
    {
        ESP_LOGE(TAG, "Data array length %d not divisible by data packet length %d", len, DATA_PACKET_SIZE);
        return 0;
    }

    // Start of frame delimiter test.
    if (data[0] != START_CHARACTER_1 || data[1] != START_CHARACTER_2 || ((data[2] << 8) + data[3]) != FRAME_LEN)
    {
        ESP_LOGE(TAG, "Frame delimeter test fail!");
        return 0;
    }

    // Checksum test
    int checksum = 0, checksum_h, checksum_l;
    for (int i = 0; i < 30; i++)
        checksum += data[i];
    checksum_h = (checksum >> 8) & 0xFF;
    checksum_l = checksum & 0xFF;
    if (data[CHECK_CODE_HI_INDEX] != checksum_h || data[CHECK_CODE_LOW_INDEX] != checksum_l)
    {
        ESP_LOGE(TAG, "Checksum test fail!");
        return 0;
    }

    // Reading data
    reading->pm1_0_std = (data[4] << 8) + data[5];
    reading->pm2_5_std = (data[6] << 8) + data[7];
    reading->pm10_std = (data[8] << 8) + data[9];
    reading->pm1_0_atm = (data[10] << 8) + data[11];
    reading->pm2_5_atm = (data[12] << 8) + data[13];
    reading->pm10_atm = (data[14] << 8) + data[15];

    return 1;
}

void pms5003_normal_mode(pms5003_config_t *inst)
{
    // Set pms set pin high to put into normal working mode.
    gpio_set_level(inst->set_pin, 1);
}

void pms5003_sleep_mode(pms5003_config_t *inst)
{
    // Set pms set pin low to put into sleeping mode.
    gpio_set_level(inst->set_pin, 0);
}

void pms5003_reset(pms5003_config_t *inst)
{
    // Set pms reset pin low to reset device.
    gpio_set_level(inst->reset_pin, 0);
    // Give sensor some time to reset.
    vTaskDelay(pdMS_TO_TICKS(100));
    // Set reset pin back high.
    gpio_set_level(inst->reset_pin, 1);
}

void pms5003_print_measurement(pms5003_measurement_t *reading)
{
    printf("PM1.0 concentration in standard material:       %d ug/m^3\n", reading->pm1_0_std);
    printf("PM2.5 concentration in standard material:       %d ug/m^3\n", reading->pm2_5_std);
    printf("PM10  concentration in standard material:       %d ug/m^3\n", reading->pm10_std);
    printf("PM1.0 concentration in atmospheric environment: %d ug/m^3\n", reading->pm1_0_atm);
    printf("PM2.5 concentration in atmospheric environment: %d ug/m^3\n", reading->pm2_5_atm);
    printf("PM10  concentration in atmospheric environment: %d ug/m^3\n", reading->pm10_atm);
    printf("\n");
}

