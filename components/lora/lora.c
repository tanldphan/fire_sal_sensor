#include "include/lora.h"

static const char* TAG = "LORA";

static spi_device_handle_t _spi;
static int _implicit;
static long _frequency;
static int _send_packet_lost = 0;
static int _cr = 0;
static int _sbw = 0;
static int _sf = 0;


void lora_write_reg (int reg, int val)
{
    uint8_t out[2] = { 0x80 | reg, val };
    uint8_t in[2];

    spi_transaction_t t = { .flags = 0, .length = 8 * sizeof (out), .tx_buffer = out, .rx_buffer = in };
    spi_device_transmit (_spi, &t);
}


void lora_write_reg_buffer (int reg, uint8_t* val, int len)
{
    uint8_t* out;
    out = (uint8_t*) malloc (len + 1);
    out[0] = 0x80 | reg;
    for (int i = 0; i < len; i++)
    {
        out[i + 1] = val[i];
    }

    spi_transaction_t t = { .flags = 0, .length = 8 * (len + 1), .tx_buffer = out, .rx_buffer = NULL };
    spi_device_transmit (_spi, &t);
    free (out);
}


int lora_read_reg (int reg)
{
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2];

    spi_transaction_t t = { .flags = 0, .length = 8 * sizeof (out), .tx_buffer = out, .rx_buffer = in };
    spi_device_transmit (_spi, &t);
    return in[1];
}


void lora_read_reg_buffer (int reg, uint8_t* val, int len)
{
    uint8_t* out;
    uint8_t* in;
    out = (uint8_t*) malloc (len + 1);
    in = (uint8_t*) malloc (len + 1);
    out[0] = reg;
    for (int i = 0; i < len; i++)
    {
        out[i + 1] = 0xff;
    }

    spi_transaction_t t = { .flags = 0, .length = 8 * (len + 1), .tx_buffer = out, .rx_buffer = in };
    spi_device_transmit (_spi, &t);

    for (int i = 0; i < len; i++)
    {
        val[i] = in[i + 1];
    }
    free (out);
    free (in);
}


int lora_init (void)
{
    esp_err_t ret;
    gpio_reset_pin (LORA_RST_GPIO);
    gpio_set_direction (LORA_RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin (LORA_CS_GPIO);
    gpio_set_direction (LORA_CS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level (LORA_CS_GPIO, 1);

    spi_bus_config_t bus = { .miso_io_num = LORA_MISO_GPIO,
                             .mosi_io_num = LORA_MOSI_GPIO,
                             .sclk_io_num = LORA_SCK_GPIO,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 0 };
    ret = spi_bus_initialize (LORA_SPI_NUM, &bus, SPI_DMA_CH_AUTO);
    assert (ret == ESP_OK);

    spi_device_interface_config_t dev = { .clock_speed_hz = 9000000,
                                          .mode = 0,
                                          .spics_io_num = LORA_CS_GPIO,
                                          .queue_size = 7,
                                          .flags = 0,
                                          .pre_cb = NULL };
    ret = spi_bus_add_device (LORA_SPI_NUM, &dev, &_spi);
    assert (ret == ESP_OK);
    lora_reset ();

    // Check Version
    uint8_t version;
    uint8_t i = 0;
    while (i++ < TIMEOUT_RESET)
    {
        version = lora_read_reg (REG_VERSION);
        ESP_LOGD (TAG, "version=0x%02x", version);
        if (version == 0x12)
            break;
        vTaskDelay (2);
    }
    ESP_LOGD (TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
    if (i == TIMEOUT_RESET + 1)
        return 0; // Illegal version
    // at the end of the loop above, the max value i can reach is TIMEOUT_RESET
    // + 1
    // assert(i < TIMEOUT_RESET + 1);

    // Default configuration.
    lora_sleep ();
    lora_write_reg (REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg (REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg (REG_LNA, lora_read_reg (REG_LNA) | 0x03);
    lora_write_reg (REG_MODEM_CONFIG_3, 0x04);
    lora_set_tx_power (17);

    lora_set_frequency (433e6); // 433MHz
    lora_enable_crc ();
    lora_set_coding_rate (1);
    lora_set_bandwidth (7);
    lora_set_spreading_factor (7);
    lora_idle ();

    return 1;
}


void lora_send_packet (uint8_t* buf, int size)
{
    // Transfer data to radio.
    lora_idle ();
    lora_write_reg (REG_FIFO_ADDR_PTR, 0);

#if BUFFER_IO
    lora_write_reg_buffer (REG_FIFO, buf, size);
#else
    for (int i = 0; i < size; i++)
        lora_write_reg (REG_FIFO, *buf++);
#endif

    lora_write_reg (REG_PAYLOAD_LENGTH, size);

    // Start transmission and wait for conclusion.
    lora_write_reg (REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
#if 0
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);
#endif
    int loop = 0;
    int max_retry;
    if (_sbw < 2)
    {
        max_retry = 500;
    }
    else if (_sbw < 4)
    {
        max_retry = 250;
    }
    else if (_sbw < 6)
    {
        max_retry = 125;
    }
    else if (_sbw < 8)
    {
        max_retry = 60;
    }
    else
    {
        max_retry = 30;
    }
    ESP_LOGD (TAG, "_sbw=%d max_retry=%d", _sbw, max_retry);
    while (1)
    {
        int irq = lora_read_reg (REG_IRQ_FLAGS);
        ESP_LOGD (TAG, "lora_read_reg=0x%x", irq);
        if ((irq & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK)
            break;
        loop++;
        if (loop == max_retry)
            break;
        vTaskDelay (2);
    }
    if (loop == max_retry)
    {
        _send_packet_lost++;
        ESP_LOGE (TAG, "lora_send_packet Fail");
    }
    lora_write_reg (REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}


int lora_receive_packet (uint8_t* buf, int size)
{
    int len = 0;

    // Check interrupts.
    int irq = lora_read_reg (REG_IRQ_FLAGS);
    lora_write_reg (REG_IRQ_FLAGS, irq);
    if ((irq & IRQ_RX_DONE_MASK) == 0)
        return 0;
    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
        return 0;

    // Find packet size.
    if (_implicit)
        len = lora_read_reg (REG_PAYLOAD_LENGTH);
    else
        len = lora_read_reg (REG_RX_NB_BYTES);

    // Transfer data from radio.
    lora_idle ();
    lora_write_reg (REG_FIFO_ADDR_PTR, lora_read_reg (REG_FIFO_RX_CURRENT_ADDR));
    if (len > size)
        len = size;
#if BUFFER_IO
    lora_read_reg_buffer (REG_FIFO, buf, len);
#else
    for (int i = 0; i < len; i++)
        *buf++ = lora_read_reg (REG_FIFO);
#endif

    return len;
}


// Configure power level 2-17, from least to most power
void lora_set_tx_power (int level)
{
    // RF9x module uses PA_BOOST pin
    if (level < 2)
        level = 2;
    else if (level > 17)
        level = 17;
    lora_write_reg (REG_PA_CONFIG, PA_BOOST | (level - 2));
}


void lora_set_frequency (long frequency)
{
    _frequency = frequency;
    uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
    lora_write_reg (REG_FRF_MSB, (uint8_t) (frf >> 16));
    lora_write_reg (REG_FRF_MID, (uint8_t) (frf >> 8));
    lora_write_reg (REG_FRF_LSB, (uint8_t) (frf >> 0));
}


// Set spreading factor 6-12
void lora_set_spreading_factor (int sf)
{
    if (sf < 6)
        sf = 6;
    else if (sf > 12)
        sf = 12;
    if (sf == 6)
    {
        lora_write_reg (REG_DETECTION_OPTIMIZE, 0xc5);
        lora_write_reg (REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        lora_write_reg (REG_DETECTION_OPTIMIZE, 0xc3);
        lora_write_reg (REG_DETECTION_THRESHOLD, 0x0a);
    }
    lora_write_reg (REG_MODEM_CONFIG_2, (lora_read_reg (REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    _sf = sf;
}


int lora_get_spreading_factor (void) { return (lora_read_reg (REG_MODEM_CONFIG_2) >> 4); }


void lora_set_dio_mapping (int dio, int mode)
{
    if (dio < 4)
    {
        int _mode = lora_read_reg (REG_DIO_MAPPING_1);
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
        lora_write_reg (REG_DIO_MAPPING_1, _mode);
        ESP_LOGD (TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
    }
    else if (dio < 6)
    {
        int _mode = lora_read_reg (REG_DIO_MAPPING_2);
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
        ESP_LOGD (TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
        lora_write_reg (REG_DIO_MAPPING_2, _mode);
    }
}


int lora_get_dio_mapping (int dio)
{
    if (dio < 4)
    {
        int _mode = lora_read_reg (REG_DIO_MAPPING_1);
        ESP_LOGD (TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
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
        int _mode = lora_read_reg (REG_DIO_MAPPING_2);
        ESP_LOGD (TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
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


void lora_set_bandwidth (int sbw)
{
    // Set Signal bandwidth (0 to 9)
    if (sbw < 10)
    {
        lora_write_reg (REG_MODEM_CONFIG_1, (lora_read_reg (REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
        _sbw = sbw;
    }
}


int lora_get_bandwidth (void)
{
    // int bw;
    // bw = lora_read_reg(REG_MODEM_CONFIG_1) & 0xf0;
    // ESP_LOGD(TAG, "bw=0x%02x", bw);
    // bw = bw >> 4;
    // return bw;
    return ((lora_read_reg (REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}


void lora_set_coding_rate (int cr)
{
    // Set Coding Rate (1 to 4)
    // if (denominator < 5) denominator = 5;
    // else if (denominator > 8) denominator = 8;

    // int cr = denominator - 4;
    if (cr < 1)
        cr = 1;
    else if (cr > 4)
        cr = 4;
    lora_write_reg (REG_MODEM_CONFIG_1, (lora_read_reg (REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
    _cr = cr;
}


int lora_get_coding_rate (void) { return ((lora_read_reg (REG_MODEM_CONFIG_1) & 0x0E) >> 1); }


void lora_set_preamble_length (long length)
{
    lora_write_reg (REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
    lora_write_reg (REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
}


long lora_get_preamble_length (void)
{
    long preamble;
    preamble = lora_read_reg (REG_PREAMBLE_MSB) << 8;
    preamble = preamble + lora_read_reg (REG_PREAMBLE_LSB);
    return preamble;
}


void lora_explicit_header_mode (void)
{
    _implicit = 0;
    lora_write_reg (REG_MODEM_CONFIG_1, lora_read_reg (REG_MODEM_CONFIG_1) & 0xfe);
}


void lora_implicit_header_mode (int size)
{
    _implicit = 1;
    lora_write_reg (REG_MODEM_CONFIG_1, lora_read_reg (REG_MODEM_CONFIG_1) | 0x01);
    lora_write_reg (REG_PAYLOAD_LENGTH, size);
}


void lora_set_sync_word (int sw) { lora_write_reg (REG_SYNC_WORD, sw); }


void lora_idle (void) { lora_write_reg (REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY); }


// Low power consumption and FIFO is lost.
void lora_sleep (void) { lora_write_reg (REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP); }


void lora_reset (void)
{
    gpio_set_level (LORA_RST_GPIO, 0);
    vTaskDelay (pdMS_TO_TICKS (1));
    gpio_set_level (LORA_RST_GPIO, 1);
    vTaskDelay (pdMS_TO_TICKS (10));
}


// Incoming packets will be received.
void lora_receive (void) { lora_write_reg (REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS); }

void lora_enable_crc (void)
{
    lora_write_reg (REG_MODEM_CONFIG_2, lora_read_reg (REG_MODEM_CONFIG_2) | 0x04);
}


void lora_disable_crc (void)
{
    lora_write_reg (REG_MODEM_CONFIG_2, lora_read_reg (REG_MODEM_CONFIG_2) & 0xfb);
}


int lora_received (void)
{
    if (lora_read_reg (REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK)
        return 1;
    return 0;
}


int lora_get_irq (void) { return (lora_read_reg (REG_IRQ_FLAGS)); }


int lora_packet_lost (void) { return (_send_packet_lost); }


int lora_packet_rssi (void)
{
    return (lora_read_reg (REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}


float lora_packet_snr (void) { return ((int8_t) lora_read_reg (REG_PKT_SNR_VALUE)) * 0.25; }


void lora_close (void)
{
    lora_sleep ();
    //   close(__spi);  FIXME: end hardware features after lora_close
    //   close(__cs);
    //   close(__rst);
    //   _spi = -1;
    //   __cs = -1;
    //   __rst = -1;
}


void lora_dump_registers (void)
{
    int i;
    printf ("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    for (i = 0; i < 0x40; i++)
    {
        printf ("%02X ", lora_read_reg (i));
        if ((i & 0x0f) == 0x0f)
            printf ("\n");
    }
    printf ("\n");
}
