#ifndef __BME_H__
#define __BME_H__

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include <stdio.h>

// I2C stuff
#define BME680_I2C_DEV_ADDR 0x77
#define BME680_I2C_PORT_NUM 0
#define BME680_I2C_SCL_PIN 2
#define BME680_I2C_SDA_PIN 41
#define BME680_I2C_CLK_SPEED 100000
// Reset
#define BME680_RESET_REG 0xE0
#define BME680_RESET_DEFAULT_VAL 0x00
#define BME680_SOFT_RESET_VAL 0xB6
// Status
#define BME680_EAS_STATUS_REG 0x1D
#define BME680_NEW_DATA_MASK 0b10000000
#define BME680_GAS_MEASURING_MASK 0b01000000
#define BME680_MEASURING_MASK 0b00100000
#define BME680_GAS_MEAS_INDEX_MASK 0b00001111
// Oversampling
#define BME680_CTRL_MEAS_REG 0x74
#define BME680_OSRS_T_MASK 0b11100000
#define BME680_OSRS_P_MASK 0b00011100
#define BME680_OSRS_X1_VAL 0b001
#define BME680_OSRS_X2_VAL 0b010
#define BME680_OSRS_X4_VAL 0b011
#define BME680_OSRS_X8_VAL 0b100
#define BME680_OSRS_X16_VAL 0b101
#define BME680_CTRL_HUM_REG 0x72
#define BME680_OSRS_H_MASK 0b00000111
// Power modes
#define BME680_MODE_MASK 0b00000011
#define BME680_MODE_FORCE_SET 0b00000001
#define BME680_MODE_SLEEP_SET 0b00000000
// IIR Filter
#define BME680_CONFIG_REG 0x75
#define BME680_FILTER_MASK 0b00011100
#define BME680_FILTER_127_VAL 0b111
// Gas config
#define BME680_CTRL_GAS_REG 0x71
#define BME680_NB_CONV_MASK 0b00001111
#define BME680_RUN_GAS_MASK 0b00010000
#define BME680_GAS_ENABLE_VAL 1
#define BME680_GAS_DISABLE_VAL 0
#define BME680_GAS_WAIT_REG 0x64
#define BME680_GAS_WAIT_LOAD_MASK 0b00111111
#define BME680_GAS_WAIT_MUL_MASK 0b11000000
#define BME680_GAS_WAIT_MUL_X1_VAL 0b00
#define BME680_GAS_WAIT_MUL_X4_VAL 0b01
#define BME680_GAS_WAIT_MUL_X16_VAL 0b10
#define BME680_GAS_WAIT_MUL_X64_VAL 0b11
// Hot plate
#define BME680_REST_HEAT_REG 0x5A
// Cofficients and ADC values
#define BME680_COFF_SET_1_ADDR 0x8A
#define BME680_COFF_SET_1_LEN 23
#define BME680_COFF_SET_2_ADDR 0xE1
#define BME680_COFF_SET_2_LEN 14
#define BME680_COFF_SET_3_ADDR 0x00
#define BME680_COFF_SET_3_LEN 5
#define BME680_ADC_VALUES_ADDR 0x1F
#define BME680_ADC_VALUES_LEN 7
#define BME680_GAS_VALUES_ADDR 0x2A
#define BME680_GAS_VALUES_LEN 2


typedef struct bme680_measurement
{
    /*
    int32_t temp_comp;
    int32_t pres_comp;
    int32_t humd_comp;
    int32_t gas_comp;
    */
    double temp_comp; // Temperature compensated in Celsius
    double pres_comp; // Pressure compensated in Pascal
    double humd_comp; // Humidity compensated in %RH
    double gas_comp;  // Gas resistance compensated in Ohms
} bme680_measurement_t;

typedef struct adc_val
{
    uint32_t temp_adc;
    uint8_t temp_adcm;
    uint8_t temp_adcl;
    uint8_t temp_adclx;
    uint32_t pres_adc;
    uint8_t pres_adcm;
    uint8_t pres_adcl;
    uint8_t pres_adclx;
    uint32_t humd_adc;
    uint8_t humd_adcm;
    uint8_t humd_adcl;
    uint16_t gas_adc;
    uint8_t gas_adcm;
    uint8_t gas_adcl;
    uint8_t gas_range;
} adc_val_t;

typedef struct t_coff
{
    uint16_t par_t1;
    uint8_t par_t1m;
    uint8_t par_t1l;
    int16_t par_t2;     // changed from uint16_t to int16_t
    uint8_t par_t2m;
    uint8_t par_t2l;
    int8_t par_t3;      // changed from uint8_t to int8_t  
    uint32_t temp_adc;
    uint8_t temp_adcm;
    uint8_t temp_adcl;
    uint8_t temp_adclx;
} t_coff_t;

typedef struct p_coff
{
    uint16_t par_p1;
    uint8_t par_p1m;
    uint8_t par_p1l;
    int16_t par_p2;     // changed from uint16_t to int16_t
    uint8_t par_p2m;
    uint8_t par_p2l;
    int8_t par_p3;      // changed from uint8_t to int8_t
    int16_t par_p4;     // changed from uint16_t to int16_t
    uint8_t par_p4m;
    uint8_t par_p4l;
    int16_t par_p5;     // changed from uint16_t to int16_t
    uint8_t par_p5m;
    uint8_t par_p5l;
    int8_t par_p6;      // changed from uint8_t to int8_t
    int8_t par_p7;      // changed from uint8_t to int8_t
    int16_t par_p8;
    uint8_t par_p8m;
    uint8_t par_p8l;
    int16_t par_p9;     // changed from uint16_t to int16_t
    uint8_t par_p9m;
    uint8_t par_p9l;
    uint8_t par_p10;
    uint32_t press_adc;
    uint8_t press_adcm;
    uint8_t press_adcl;
    uint8_t press_adclx;
} p_coff_t;

typedef struct h_coff
{
    uint16_t par_h1;
    uint8_t par_h1m;
    uint8_t par_h1l;
    int16_t par_h2;     // changed from uint16_t to int16_t
    uint8_t par_h2m;
    uint8_t par_h2l;
    int8_t par_h3;      // changed from uint8_t to int8_t
    int8_t par_h4;      // changed from uint8_t to int8_t
    int8_t par_h5;      // changed from uint8_t to int8_t
    uint8_t par_h6;
    int8_t par_h7;      // changed from uint8_t to int8_t
    uint16_t humd_adc;
    uint8_t humd_adcm;
    uint8_t humd_adcl;
} h_coff_t;

typedef struct g_coff
{
    int8_t par_g1;      // changed from uint8_t to int8_t
    int16_t par_g2;     // changed from uint16_t to int16_t
    uint8_t par_g2m;
    uint8_t par_g2l;
    int8_t par_g3;      // changed from uint8_t to int8_t
    uint8_t res_heat_range;
    int8_t res_heat_value;      // changed from uint8_t to int8_t   
    uint8_t gas_range;
    int8_t range_switching_error;   // changed from uint8_t to int8_t
} g_coff_t;


void bme680_init (_Bool first_init);
void bme680_i2c_init (void);
void bme680_i2c_transmit (uint8_t reg_addr, uint8_t reg_data);
void bme680_i2c_receive (uint8_t reg_addr, uint8_t* reg_data, uint8_t size);
void bme680_delay_ms (uint32_t delay);
void set_reg (uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data);
void get_calib ();
void get_adc ();
void calc_res_heat ();
void calc_temp (bme680_measurement_t* readings);
void calc_pres (bme680_measurement_t* readings);
void calc_humd (bme680_measurement_t* readings);
void calc_gas (bme680_measurement_t* readings);
void bme680_make_measurement (bme680_measurement_t* readings);

#endif
