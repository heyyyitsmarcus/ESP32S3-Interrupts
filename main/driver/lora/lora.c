#include "lora.h"

/*
 * Register definitions
 */
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

/*
 * PA configuration
 */
#define PA_BOOST 0x80

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

#define TIMEOUT_RESET 100

#define TAG "LORA"

static int __implicit;
static long __frequency;

// use buffer io
// A little faster
#define BUFFER_IO 1


// LoRa configuration.
void lora_config(lora_config_t* lora_config, long initial_freq, int coding_rate,
                 int bandwidth, int spread_factor)
{
    if (lora_init(lora_config) == 0)
    {
        while (1)
        {
            ESP_LOGE(pcTaskGetName(NULL), "Module does not work!\n");
            vTaskDelay(1);
        }
    }

    lora_set_frequency(initial_freq, lora_config);

    lora_set_coding_rate(coding_rate, lora_config);
    lora_set_bandwidth(bandwidth, lora_config);
    lora_set_spreading_factor(spread_factor, lora_config);

    lora_explicit_header_mode(lora_config);
    lora_enable_crc(lora_config);

    ESP_LOGI(pcTaskGetName(NULL), "LoRa Configured\n");
}

/**
 * Write a value to a register.  * @param reg Register index.
 * @param val Value to write.
 */
void lora_write_reg(int reg, int val, lora_config_t* lora_config)
{
   uint8_t out[2] = {0x80 | reg, val};
   uint8_t in[2];

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * sizeof(out),
       .tx_buffer = out,
       .rx_buffer = in};

   spi_device_transmit(lora_config->device_handle, &t);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
void lora_write_reg_buffer(int reg, uint8_t *val, int len, lora_config_t* lora_config)
{
   uint8_t *out;
   out = (uint8_t *)malloc(len + 1);
   out[0] = 0x80 | reg;
   for (int i = 0; i < len; i++)
   {
      out[i + 1] = val[i];
   }

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * (len + 1),
       .tx_buffer = out,
       .rx_buffer = NULL};

   spi_device_transmit(lora_config->device_handle, &t);

   free(out);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int lora_read_reg(int reg, lora_config_t* lora_config)
{
   uint8_t out[2] = {reg, 0xff};
   uint8_t in[2];

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * sizeof(out),
       .tx_buffer = out,
       .rx_buffer = in};

   spi_device_transmit(lora_config->device_handle, &t);

   return in[1];
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
void lora_read_reg_buffer(int reg, uint8_t *val, int len, lora_config_t* lora_config)
{
   uint8_t *out;
   uint8_t *in;
   out = (uint8_t *)malloc(len + 1);
   in = (uint8_t *)malloc(len + 1);
   out[0] = reg;
   for (int i = 0; i < len; i++)
   {
      out[i + 1] = 0xff;
   }

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * (len + 1),
       .tx_buffer = out,
       .rx_buffer = in};

   spi_device_transmit(lora_config->device_handle, &t);

   for (int i = 0; i < len; i++)
   {
      val[i] = in[i + 1];
   }
   free(out);
   free(in);
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(lora_config_t* lora_config)
{
   gpio_set_level(lora_config->rst_gpio, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(lora_config->rst_gpio, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(lora_config_t* lora_config)
{
   __implicit = 0;
   // The "& 0xfe" preserves bits 1-7 and makes the bit 0 = 0, thus resetting 
   // ImplicitHeaderModeOn resulting in Explicit Header  mode being used.
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1, lora_config) & 0xfe, lora_config);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(int size, lora_config_t* lora_config)
{
   __implicit = 1;
   // The "| 0x01" preserves bits 1-7 and makes bit 0 = 1, thus setting
   // Implicit Header mode.
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1, lora_config) | 0x01, lora_config);
   // Write the payload length in bytes.
   // This register must be set in implicit header mode for the expected
   // packet length.
   lora_write_reg(REG_PAYLOAD_LENGTH, size, lora_config);
}

/**
 * Sets the radio transceiver in standby mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_standby(lora_config_t* lora_config)
{
   // Accesses the operation mode register and writes 1 into the LongRangeMode
   // variable (can only be modified in sleep mode) setting the lora mode, and 
   // writes 001 into the Mode variable putting the chip in standby mode.
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY, lora_config);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(lora_config_t* lora_config)
{
   // Accesses the operation mode register and writes 1 into the LongRangeMode
   // variable (can only be modified in sleep mode) setting the LoRa mode, and 
   // writes 000 into the Mode variable putting the chip in sleep mode.
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP, lora_config);
}

/**
 * Sets the radio transceiver in receive continuous mode.
 * Incoming packets will be received.
 */
void lora_receive_continuous(lora_config_t* lora_config)
{
   // Accesses the operation mode register and writes 1 into the LongRangeMode
   // variable (can only be modified in sleep mode) setting the LoRa mode, and 
   // writes 101 into the Mode variable putting the chip in Receive continuous mode.
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS, lora_config);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
// TODO: come back to this.
void lora_set_tx_power(int level, lora_config_t* lora_config)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2)
      level = 2;
   else if (level > 17)
      level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2), lora_config);
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency, lora_config_t* lora_config)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
   // Write RF carrier frequency to its 3 dedicated memory addresses.
   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16), lora_config);
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8), lora_config);
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0), lora_config);
}

/**
 * Set spreading factor.
 * Expressed as a base-2 logarithm (log_2(sf))
 * ie. 6 -> 64  chips / symbol
 *     7 -> 128 chips / symbol
 *     ...
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int sf, lora_config_t* lora_config)
{
   if (sf < 6)
      sf = 6;
   else if (sf > 12)
      sf = 12;
    
   // Set DetectionOptimize and DetectionThreshold variables
   // according to selected spreading factor.
   if (sf == 6)
   {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5, lora_config);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c, lora_config);
   }
   else
   {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3, lora_config);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a, lora_config);
   }
   // Preserve bits 0-3 and write spreading factor to bits 4-7.
   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2, lora_config) & 0x0f) | ((sf << 4) & 0xf0), lora_config);
}

/**
 * Get spreading factor.
 */
int lora_get_spreading_factor(lora_config_t* lora_config)
{
   // Reads bits 4-7, corresponding to the SpreadingFactor variable.
   return (lora_read_reg(REG_MODEM_CONFIG_2, lora_config) >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
// TODO: come back to this.
void lora_set_dio_mapping(int dio, int mode, lora_config_t* lora_config)
{

   // Set mapping of DIO0 to DIO3
   if (dio < 4)
   {
      int _mode = lora_read_reg(REG_DIO_MAPPING_1, lora_config);
      if (dio == 0)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 1)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      else if (dio == 2)
      {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      }
      else if (dio == 3)
      {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }
      lora_write_reg(REG_DIO_MAPPING_1, _mode, lora_config);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
   } // Set mapping of DIO4 and DIO5
   else if (dio < 6)
   {
      int _mode = lora_read_reg(REG_DIO_MAPPING_2, lora_config);
      if (dio == 4)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 5)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      lora_write_reg(REG_DIO_MAPPING_2, _mode, lora_config);
   }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int lora_get_dio_mapping(int dio, lora_config_t* lora_config)
{
   if (dio < 4)
   {
      int _mode = lora_read_reg(REG_DIO_MAPPING_1, lora_config);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
      if (dio == 0)
      {
         return ((_mode >> 6) & 0x03);
      }
      else if (dio == 1)
      {
         return ((_mode >> 4) & 0x03);
      }
      else if (dio == 2)
      {
         return ((_mode >> 2) & 0x03);
      }
      else if (dio == 3)
      {
         return (_mode & 0x03);
      }
   }
   else if (dio < 6)
   {
      int _mode = lora_read_reg(REG_DIO_MAPPING_2, lora_config);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      if (dio == 4)
      {
         return ((_mode >> 6) & 0x03);
      }
      else if (dio == 5)
      {
         return ((_mode >> 4) & 0x03);
      }
   }
   return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 * Table of values:
 *  0000 -> 7.8 kHz
 *  0001 -> 10.4 kHz
 *  0010 -> 15.6 kHz
 *  0011 -> 20.8 kHz
 *  0100 -> 31.25 kHz
 *  0101 -> 41.7 kHz
 *  0110 -> 62.5 kHz
 *  0111 -> 125 kHz
 *  1000 -> 250 kHz
 *  1001 -> 500 kHz
 */
void lora_set_bandwidth(int sbw, lora_config_t* lora_config)
{
   if (sbw < 10)
   {
      // Preserve bits 0-3 and set the Bw variable.
      lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1, lora_config) & 0x0f) | (sbw << 4), lora_config);
   }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int lora_get_bandwidth(lora_config_t* lora_config)
{
   // Read Bw variable.
   return ((lora_read_reg(REG_MODEM_CONFIG_1, lora_config) & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void lora_set_coding_rate(int denominator, lora_config_t* lora_config)
{
   if (denominator < 5)
      denominator = 5;
   else if (denominator > 8)
      denominator = 8;

   int cr = denominator - 4;
   // Preserve bit 0 and bits 4-7, sets CodingRate variable.
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1, lora_config) & 0xf1) | (cr << 1), lora_config);
}

/**
 * Get coding rate
 */
int lora_get_coding_rate(lora_config_t* lora_config)
{
   // Read CodingRate variable.
   return ((lora_read_reg(REG_MODEM_CONFIG_1, lora_config) & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(long length, lora_config_t* lora_config)
{
   // Writes the size of the preamble to its associated registers.
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8), lora_config);
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0), lora_config);
}

/**
 * Get the size of preamble.
 */
long lora_get_preamble_length(lora_config_t* lora_config)
{
   long preamble;
   preamble = lora_read_reg(REG_PREAMBLE_MSB, lora_config) << 8;
   preamble = preamble + lora_read_reg(REG_PREAMBLE_LSB, lora_config);
   return preamble;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void lora_set_sync_word(int sw, lora_config_t* lora_config)
{
   lora_write_reg(REG_SYNC_WORD, sw, lora_config);
}

/**
 * Enable CRC generation and check on payload.
 */
void lora_enable_crc(lora_config_t* lora_config)
{
   // Sets bit 2 to 1, enabling RxPayloadCrcOn variable.
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2, lora_config) | 0x04, lora_config);
}

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(lora_config_t* lora_config)
{
   // Sets bit 2 to 0, disabling RxPayloadCrcOn
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2, lora_config) & 0xfb, lora_config);
}

/**
 * Perform hardware initialization.
 */
int lora_init(lora_config_t* lora_config) 
{
    /*
     * Configure CPU hardware to communicate with the radio chip
     */
    gpio_reset_pin(lora_config->rst_gpio);
    gpio_set_direction(lora_config->rst_gpio, GPIO_MODE_OUTPUT);
    gpio_reset_pin(lora_config->cs_gpio);
    gpio_set_direction(lora_config->cs_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(lora_config->cs_gpio, 1);

#if USE_INTERRUPTS
    gpio_reset_pin(lora_config->irq_gpio);
    gpio_set_direction(lora_config->irq_gpio, GPIO_MODE_INPUT);
    gpio_set_level(lora_config->irq_gpio, 0);
    gpio_set_intr_type(lora_config->irq_gpio, GPIO_INTR_HIGH_LEVEL);
    gpio_install_isr_service(0); 
    gpio_isr_handler_add(lora_config->irq_gpio, lora_config->isr, NULL);
    gpio_intr_enable(lora_config->irq_gpio);
#endif

    spi_bus_config_t bus = {
        .miso_io_num = lora_config->miso_gpio,
        .mosi_io_num = lora_config->mosi_gpio,
        .sclk_io_num = lora_config->sck_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};

    spi_bus_initialize(lora_config->host_id, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = lora_config->cs_gpio,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL};
    spi_bus_add_device(lora_config->host_id, &dev, &lora_config->device_handle);
    /*
     * Perform hardware reset.
     */
    lora_reset(lora_config);

    /*
     * Check version.
     */
    uint8_t version;
    uint8_t i = 0;
    while (i++ < TIMEOUT_RESET)
    {
        version = lora_read_reg(REG_VERSION, lora_config);
        ESP_LOGD(TAG, "version=0x%02x", version);
        if (version == 0x12)
            break;
        vTaskDelay(2);
    }
    ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
    if (i == TIMEOUT_RESET + 1)
        return 0; // Illegal version
                  // assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

    /*
     * Default configuration.
     */
    lora_sleep(lora_config);
    // Resets FifoRxBaseAddr variable.
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0, lora_config);
    // Writes the base address in FIFO data buffer to 0 for the TX modulator
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0, lora_config);
    // Sets the LnaBoostHf variable to 11 for Boost on, 150% LNA current.
    lora_write_reg(REG_LNA, lora_read_reg(REG_LNA, lora_config) | 0x03, lora_config);
    // Sets the AgcAutoOn variable to 1 for LNA gain set by register LnaGain.
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04, lora_config);
    lora_set_tx_power(17, lora_config);

    lora_standby(lora_config);

    return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, int size, lora_config_t* lora_config)
{
   /*
    * Transfer data to radio.
    */
   lora_standby(lora_config);
   // Sets the SPI interface address pointer in FIFO data buffer to address 0.
   lora_write_reg(REG_FIFO_ADDR_PTR, 0, lora_config);

#if BUFFER_IO
   // Write buffer to FIFO.
   lora_write_reg_buffer(REG_FIFO, buf, size, lora_config);
#else
   for (int i = 0; i < size; i++)
      lora_write_reg(REG_FIFO, *buf++);
#endif
   // Sets the PayloadLength variable to the size of the buffer in bytes.
   lora_write_reg(REG_PAYLOAD_LENGTH, size, lora_config);

   /*
    * Start transmission and wait for conclusion.
    */
   // Sets Mode variable to 011 -> Transmit (TX).
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX, lora_config);
   // Reads the TxDone variable signifying the FIFO Payload transmission complete 
   // interupt. Delay until TxDone IRF flag is set.
   while ((lora_read_reg(REG_IRQ_FLAGS, lora_config) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);
    
   // Clear the TxDone IRQ by writing a 1 to it..
   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK, lora_config);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int lora_receive_packet(uint8_t *buf, int size, lora_config_t* lora_config)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS, lora_config);
   // Clear the set IRQ's by writing 1's to those set.
   lora_write_reg(REG_IRQ_FLAGS, irq, lora_config);
   // Return if packet reception isn't complete.
   if ((irq & IRQ_RX_DONE_MASK) == 0)
      return 0;
   // Return if payload crc error interrupt is set.
   if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
      return 0;

//   // NOTE: This is unnecessary.
//   // Save the value of the RxDone IRQ flag, if it is not set then return.
//   int receive_done = lora_read_reg(REG_IRQ_FLAGS, lora_config) & IRQ_RX_DONE_MASK;
//   if (!receive_done)
//       return 0;
//   // NOTE: Not sure why this is here, commenting it our for now as it seems to have no purpose.
//   lora_write_reg(REG_IRQ_FLAGS, lora_read_reg(REG_IRQ_FLAGS, lora_config) & 0x4f, lora_config);

   /*
    * Find packet size.
    */
   if (__implicit) {
      // Read the payload length in bytes.
      len = lora_read_reg(REG_PAYLOAD_LENGTH, lora_config);
    }
   else { // Explicit mode is active.
      // Read the number of payload bytes of latest packet received.
      len = lora_read_reg(REG_RX_NB_BYTES, lora_config);
    }

   /*
    * Transfer data from radio.
    */

   // NOTE: We do not need to go into standby to read FIFO, this infact
   // breaks out of our continuous receiving mode.
   //lora_standby(lora_config);

   // Sets the SPI interface address pointer in FIFO data buffer to the starting
   // address (in the data buffer) of the last packet received.
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, lora_config), lora_config);
   // If the number of payload bytes is greater than the size of the buffer, then
   // set len to the size of the buffer so we only read what fits into the buffer.
   if (len > size)
      len = size;
#if BUFFER_IO
   // Read FIFO from FIFO address pointer up to len # of bytes.
   lora_read_reg_buffer(REG_FIFO, buf, len, lora_config);
#else
   for (int i = 0; i < len; i++)
      *buf++ = lora_read_reg(REG_FIFO, lora_config);
#endif

   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(lora_config_t* lora_config)
{
   // Check if the RxDone IRQ is set, signifying a valid receival.
   if (lora_read_reg(REG_IRQ_FLAGS, lora_config) & IRQ_RX_DONE_MASK)
      return 1;
   return 0;
}

/**
 * Returns RegIrqFlags.
 */
int lora_get_irq(lora_config_t* lora_config)
{
   return (lora_read_reg(REG_IRQ_FLAGS, lora_config));
}

/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(lora_config_t* lora_config)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE, lora_config) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(lora_config_t* lora_config)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE, lora_config)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void lora_close(lora_config_t* lora_config)
{
   lora_sleep(lora_config);
   //   close(__spi);  FIXME: end hardware features after lora_close
   //   close(__cs);
   //   close(__rst);
   //   __spi = -1;
   //   __cs = -1;
   //   __rst = -1;
}

void lora_dump_registers(lora_config_t* lora_config)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for (i = 0; i < 0x40; i++)
   {
      printf("%02X ", lora_read_reg(i, lora_config));
      if ((i & 0x0f) == 0x0f)
         printf("\n");
   }
   printf("\n");
}
