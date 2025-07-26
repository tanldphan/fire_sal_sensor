#ifndef __PMS5003_H__
#define __PMS5003_H__

#include "driver/gpio.h"
#include "driver/uart.h"

// UART Stuff
#define PMS_UART_NUM 1
#define PMS_RST_GPIO 19
#define PMS_RXD_GPIO 20
#define PMS_TXD_GPIO 21
#define PMS_SET_GPIO 47
#define BAUD_RATE 9600
// Data Frame
#define DATA_PACKET_SIZE 32
#define START_CHAR_1 0x42
#define START_CHAR_2 0x4D
#define FRAME_LEN 28
#define RX_BUFFER_SIZE 160
#define CHECK_CODE_HI_INDEX 30
#define CHECK_CODE_LOW_INDEX 31


typedef struct
{
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
} pms5003_measurement_t;

typedef enum
{
    PMS5003_OK,
    PMS5003_LENGTH_ERROR,
    PMS5003_DELIMITER_ERROR,
    PMS5003_CHKSUM_ERROR
} pms5003_status_t;


void pms5003_init (_Bool first_init);
void pms5003_make_measurement (pms5003_measurement_t* reading);
int pms5003_process_data (int len, uint8_t* data, pms5003_measurement_t* reading);
void pms5003_send_command (uint8_t cmd, uint8_t datah, uint8_t datal);
/*
Functions no longer used, set and reset pins need to be disconected to send commands to PMS5003
void pms5003_normal_mode ();
void pms5003_sleep_mode ();
void pms5003_reset ();
*/
#endif
