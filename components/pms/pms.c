#include "include/pms.h"

void pms5003_init (_Bool first_init)
{
    if (first_init)
    {
    // Initializing GPIO pins
    gpio_config_t io_conf = { .intr_type = GPIO_INTR_DISABLE,
                              .mode = GPIO_MODE_OUTPUT,
                              .pin_bit_mask = (1ULL << PMS_SET_GPIO) | (1ULL << PMS_RST_GPIO),
                              .pull_down_en = 0,
                              .pull_up_en = 0,
    };
    gpio_config (&io_conf);

    // Initializing UART interface
    uart_config_t uart_config = { .baud_rate = BAUD_RATE,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config (PMS_UART_NUM, &uart_config);
    uart_set_pin (PMS_UART_NUM, PMS_TXD_GPIO, PMS_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Size of uart RX FIFO must be greater than the size of the hardware FIFO which is 128 bytes.
    uart_driver_install (PMS_UART_NUM, RX_BUFFER_SIZE, 0, 0, NULL, 0);
    }
    /*
    no longer used, set and reset pins need to be disconected to send commands to PMS5003
    pms5003_normal_mode ();
    pms5003_reset ();
    */
    pms5003_send_command (0xE1, 0x00, 0x00);
    uart_flush (PMS_UART_NUM); 
    // Might be valuble to add logic to read the confirmation response from sensor,
    // for now the ESP just flushes the data to clear the buffer for the PMS readings */
    }



void pms5003_make_measurement (pms5003_measurement_t* reading)
{
    pms5003_send_command (0xE2, 0x00, 0x00);
    uint8_t data[RX_BUFFER_SIZE] = { 0 };
    int data_len = 0;
    // Get the length of the current data within uart controller RX FIFO buffer.
    uart_get_buffered_data_len (PMS_UART_NUM, (size_t*) &data_len);
    //printf ("Data length: %d\n", data_len);
    // Read in the returned length amount of bytes. This may be more than the data packet
    // size (32), one option is to read regardless and ignore the extra data or flush the
    // RX buffer when this happens. Choosing the first.
        data_len = uart_read_bytes (PMS_UART_NUM, (void*) data, data_len, 20);
    if (data_len > 0)
        pms5003_process_data (data_len, data, reading);
}


int pms5003_process_data (int len, uint8_t* data, pms5003_measurement_t* reading)
{
    // Length test. Is the length of the uart data a length divisible by the data packet
    // size. Some uart reads will take in two or more data packets, we will choose to only
    // read the first.
    if (len % DATA_PACKET_SIZE != 0)
    {
        return PMS5003_LENGTH_ERROR;
    }
    // Start of frame delimiter test.
    if (data[0] != START_CHAR_1 || data[1] != START_CHAR_2 || ((data[2] << 8) + data[3]) != FRAME_LEN)
    {
        return PMS5003_DELIMITER_ERROR;
    }
    // Checksum test
    int checksum = 0, checksum_h, checksum_l;
    for (int i = 0; i < 30; i++)
        checksum += data[i];
    checksum_h = (checksum >> 8) & 0xFF;
    checksum_l = checksum & 0xFF;
    if (data[CHECK_CODE_HI_INDEX] != checksum_h || data[CHECK_CODE_LOW_INDEX] != checksum_l)
    {
        return PMS5003_CHKSUM_ERROR;
    }
    // Parsing data
    reading->pm1_0_std = (data[4] << 8) + data[5];
    reading->pm2_5_std = (data[6] << 8) + data[7];
    reading->pm10_std = (data[8] << 8) + data[9];
    reading->pm1_0_atm = (data[10] << 8) + data[11];
    reading->pm2_5_atm = (data[12] << 8) + data[13];
    reading->pm10_atm = (data[14] << 8) + data[15];
    return PMS5003_OK;
}

void pms5003_send_command (uint8_t cmd, uint8_t datah, uint8_t datal)
{
    // Active mode command:     0xE1, 0x00, 0x01
    // Passive mode command:    0xE1, 0x00, 0x00
    // Read data command:       0xE2, 0x00, 0x00
    // Sleep mode command:      0xE4, 0x00, 0x00
    // Wake up command:         0xE4, 0x00, 0x01
    
    uint8_t command[7] = {START_CHAR_1, START_CHAR_2, cmd, datah, datal};
    uint16_t checksum = command[0] + command[1] + command[2] + command[3] + command[4];
    command[5] = (checksum >> 8) & 0xFF;
    command[6] = checksum & 0xFF;
    uart_write_bytes (PMS_UART_NUM, command, sizeof (command));
    //printf ("Command sent: %02X %02X %02X %02X %02X %02X %02X\n", command[0], command[1], command[2],
    //      command[3], command[4], command[5], command[6]);
    vTaskDelay (pdMS_TO_TICKS (100));
}

/*
    Functions no longer used, set and reset pins need to be disconected to send commands to PMS5003
    void pms5003_normal_mode () { gpio_set_level (PMS_SET_GPIO, 1); }
    void pms5003_sleep_mode () { gpio_set_level (PMS_SET_GPIO, 0); } 

    void pms5003_reset ()
    {
        gpio_set_level (PMS_RST_GPIO, 0);
        vTaskDelay (pdMS_TO_TICKS (100));
        gpio_set_level (PMS_RST_GPIO, 1);
        vTaskDelay (pdMS_TO_TICKS (100));
    }
*/
