#include "include/bme.h"

static i2c_master_dev_handle_t dev_handle;
static t_coff_t tcoff = { 0 };
static p_coff_t pcoff = { 0 };
static h_coff_t hcoff = { 0 };
static g_coff_t gcoff = { 0 };
static adc_val_t adc_val = { 0 };
static uint8_t res_heat = 0;


void bme680_i2c_init ()
{
    i2c_master_bus_config_t i2c_mst_config = { .clk_source = I2C_CLK_SRC_DEFAULT,
                                               .i2c_port = BME680_I2C_PORT_NUM,
                                               .scl_io_num = BME680_I2C_SCL_PIN,
                                               .sda_io_num = BME680_I2C_SDA_PIN,
                                               .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus (&i2c_mst_config, &bus_handle);

    i2c_device_config_t dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                    .device_address = BME680_I2C_DEV_ADDR,
                                    .scl_speed_hz = BME680_I2C_CLK_SPEED,
    };
    i2c_master_bus_add_device (bus_handle, &dev_cfg, &dev_handle);
}


void bme680_i2c_receive (uint8_t reg_addr, uint8_t* reg_data, uint8_t size)
{
    ESP_ERROR_CHECK (i2c_master_transmit_receive (dev_handle, &reg_addr, 1, reg_data, size, -1));
}


void bme680_i2c_transmit (uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t buf[2] = { reg_addr, reg_data };
    ESP_ERROR_CHECK (i2c_master_transmit (dev_handle, buf, 2, -1));
}


void bme680_delay_ms (uint32_t delay) { vTaskDelay (delay / portTICK_PERIOD_MS); }


void bme680_init (_Bool first_init)
{
    if(first_init)
    {
    bme680_i2c_init ();
    }

    bme680_i2c_transmit (BME680_RESET_REG, BME680_SOFT_RESET_VAL);
    bme680_delay_ms (100);

    get_calib ();
    calc_res_heat ();

    // TODO: Need to make this more elegant using bitfields
    set_reg (BME680_CTRL_MEAS_REG, BME680_OSRS_T_MASK | BME680_OSRS_P_MASK, 0b10000100);
    // set oversampling for humidity=x16: reg=0x72 data=0b00000101
    set_reg (BME680_CTRL_HUM_REG, BME680_OSRS_H_MASK, 0b00000101);
    // set IIR filter for gas conversion: reg=0x75 data=0b00011100
    set_reg (BME680_CONFIG_REG, BME680_FILTER_MASK, 0b00011100);
    // enable gas conversion and set index of heater set-point
    set_reg (BME680_CTRL_GAS_REG, BME680_NB_CONV_MASK | BME680_RUN_GAS_MASK, 0b00010000);
    // set heater on time
    set_reg (BME680_GAS_WAIT_REG, 0xff, 0b01011001);
    // set heater resistance
    set_reg (BME680_REST_HEAT_REG, 0xff, res_heat);
}


void bme680_make_measurement (bme680_measurement_t* readings)
{
    // set force mode
    set_reg (BME680_CTRL_MEAS_REG, BME680_MODE_MASK, 0b00000001);

    // check for new data
    uint8_t reg_value = 0;
    while (reg_value == 0)
    {
        bme680_i2c_receive (BME680_EAS_STATUS_REG, &reg_value, 1);
        bme680_delay_ms (100);
        reg_value &= BME680_NEW_DATA_MASK;
    }
    get_adc ();
    bme680_delay_ms (100);

    // calculate new readings
    calc_temp (readings);
    calc_pres (readings);
    calc_humd (readings);
    calc_gas (readings);
}


void get_calib ()
{
    // Get 1st Cofficients set
    uint8_t reg_addr = BME680_COFF_SET_1_ADDR;
    uint8_t coff_1[BME680_COFF_SET_1_LEN] = { 0 };
    bme680_i2c_receive (reg_addr, coff_1, sizeof (coff_1));

    // Get 2nd Cofficient set
    reg_addr = BME680_COFF_SET_2_ADDR;
    uint8_t coff_2[BME680_COFF_SET_2_LEN] = { 0 };
    bme680_i2c_receive (reg_addr, coff_2, sizeof (coff_2));

    // Get 3rd Cofficient set
    reg_addr = BME680_COFF_SET_3_ADDR;
    uint8_t coff_3[BME680_COFF_SET_3_LEN] = { 0 };
    bme680_i2c_receive (reg_addr, coff_3, sizeof (coff_3));

    // Map Cofficient data to structs
    tcoff.par_t1l = coff_2[8];
    tcoff.par_t1m = coff_2[9];
    tcoff.par_t1 = (coff_2[9] << 8) + coff_2[8];
    tcoff.par_t2l = coff_1[0];
    tcoff.par_t2m = coff_1[1];
    tcoff.par_t2 = (coff_1[1] << 8) + coff_1[0];
    tcoff.par_t3 = coff_1[2];

    pcoff.par_p1l = coff_1[4];
    pcoff.par_p1m = coff_1[5];
    pcoff.par_p1 = (coff_1[5] << 8) + coff_1[4];
    pcoff.par_p2l = coff_1[6];
    pcoff.par_p2m = coff_1[7];
    pcoff.par_p2 = (coff_1[7] << 8) + coff_1[6];
    pcoff.par_p3 = coff_1[8];
    pcoff.par_p4l = coff_1[10];
    pcoff.par_p4m = coff_1[11];
    pcoff.par_p4 = (coff_1[11] << 8) + coff_1[10];
    pcoff.par_p5l = coff_1[12];
    pcoff.par_p5m = coff_1[13];
    pcoff.par_p5 = (coff_1[13] << 8) + coff_1[12];
    pcoff.par_p6 = coff_1[15];
    pcoff.par_p7 = coff_1[14];
    pcoff.par_p8l = coff_1[18];
    pcoff.par_p8m = coff_1[19];
    pcoff.par_p8 = (coff_1[19] << 8) + coff_1[18];
    pcoff.par_p9l = coff_1[20];
    pcoff.par_p9m = coff_1[21];
    pcoff.par_p9 = (coff_1[21] << 8) + coff_1[20];
    pcoff.par_p10 = coff_1[22];

    hcoff.par_h1l = coff_2[1] & 0x01;
    hcoff.par_h1m = coff_2[2];
    hcoff.par_h1 = (coff_2[2] << 4) + (coff_2[1] & 0x01); // changed from (coff_2[2] << 8)  + (coff_2[1] & 0x01)
    hcoff.par_h2l = coff_2[1] >> 4; // 7:4
    hcoff.par_h2m = coff_2[0];
    hcoff.par_h2 = (coff_2[0] << 4) + (coff_2[1] >> 4); // changed from (coff_2[0] << 8) + (coff_2[1] >> 4)
    hcoff.par_h3 = coff_2[3];
    hcoff.par_h4 = coff_2[4];
    hcoff.par_h5 = coff_2[5];
    hcoff.par_h6 = coff_2[6];
    hcoff.par_h7 = coff_2[7];

    gcoff.par_g1 = coff_2[12];
    gcoff.par_g2l = coff_2[10];
    gcoff.par_g2m = coff_2[11];
    gcoff.par_g2 = (coff_2[11] << 8) + coff_2[10];
    gcoff.par_g3 = coff_2[13];

    gcoff.res_heat_range = (coff_3[2] >> 4) & 0x03;
    gcoff.res_heat_value = coff_3[0];
    gcoff.range_switching_error = coff_3[4];
}


void get_adc ()
{
    uint8_t adc[BME680_ADC_VALUES_LEN] = { 0 };
    bme680_i2c_receive (BME680_ADC_VALUES_ADDR, adc, sizeof (adc));
    bme680_delay_ms (100);

    adc_val.temp_adclx = adc[5] >> 4;
    adc_val.temp_adcl = adc[4];
    adc_val.temp_adcm = adc[3];
    adc_val.temp_adc = (adc[3] << 12) + (adc[4] << 4) + (adc[5] >> 4);

    adc_val.pres_adclx = adc[2] >> 4;
    adc_val.pres_adcl = adc[1];
    adc_val.pres_adcm = adc[0];
    adc_val.pres_adc = (adc[0] << 12) + (adc[1] << 4) + (adc[2] >> 4);

    adc_val.humd_adcl = adc[7];
    adc_val.humd_adcm = adc[6];
    adc_val.humd_adc = (adc[6] << 8) + adc[7];

    uint8_t gas[BME680_GAS_VALUES_LEN] = { 0 };
    bme680_i2c_receive (BME680_GAS_VALUES_ADDR, gas, sizeof (gas));

    adc_val.gas_adcm = gas[0];
    adc_val.gas_adcl = gas[1] >> 6;
    adc_val.gas_adc = (gas[0] << 2) + (gas[1] >> 6);
    adc_val.gas_range = gas[1] & 0x01;
}


void calc_res_heat ()
{
    // NOTE: For Float
    
     double var1 = ((double)gcoff.par_g1 / 16.0) + 49.0;
     double var2 = (((double)gcoff.par_g2 / 32768.0) * 0.0005) + 0.00235;
     double var3 = (double)gcoff.par_g3 / 1024.0;
     double var4 = var1 * (1.0 + (var2 * 300.0));
     double var5 = var4 + (var3 * 25.0);
     res_heat    = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + gcoff.res_heat_range)) *
                                            (1.0 / (1.0 + (gcoff.res_heat_value *0.002)))) -25));

    // NOTE: For Int
    /*
    int32_t var1 = (((int32_t) 25 * gcoff.par_g3) / 10) << 8;
    int32_t var2 = (gcoff.par_g1 + 784) * (((((gcoff.par_g2 + 154009) * 300 * 5) / 100) + 3276800) / 10);
    int32_t var3 = var1 + (var2 >> 1);
    int32_t var4 = (var3 / (gcoff.res_heat_range + 4));
    int32_t var5 = (131 * gcoff.res_heat_value) + 65536;
    int32_t res_heat_x100 = (int32_t) (((var4 / var5) - 250) * 34);

    res_heat = (uint8_t) ((res_heat_x100 + 50) / 100);
    */
}


void calc_temp (bme680_measurement_t* readings)
{
    // NOTE: For Float
    
     double var1   = (((double)adc_val.temp_adc / 16384.0) - ((double)tcoff.par_t1 / 1024.0)) *
                     (double)tcoff.par_t2;
     double var2   = ((((double)adc_val.temp_adc / 131072.0) - ((double)tcoff.par_t1 / 8192.0)) *
                      (((double)adc_val.temp_adc / 131072.0) - ((double)tcoff.par_t1 / 8192.0))) *
                     ((double)tcoff.par_t3 * 16.0);
    double t_fine = var1 + var2;
    readings->temp_comp = t_fine / 5120.0;

    // NOTE: For Int
    /*
    int32_t var1 = ((int32_t) adc_val.temp_adc >> 3) - ((int32_t) tcoff.par_t1 << 1);
    int32_t var2 = (var1 * (int32_t) tcoff.par_t2) >> 11;
    int32_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t) tcoff.par_t3 << 4)) >> 14;
    int32_t t_fine = var2 + var3;

    readings->temp_comp = ((t_fine * 5) + 128) >> 8;
    */
}


void calc_pres (bme680_measurement_t* readings)
{
    // NOTE: For Float
    
     double var1 = (readings->temp_comp * 2560.0) - 64000.0;
     double var2 = var1 * var1 * ((double)pcoff.par_p6 / 131072.0);
    
     var2 = var2 + (var1 * (double)pcoff.par_p5 * 2.0);
     var2 = (var2 / 4.0) + ((double)pcoff.par_p4 * 65536.0);
     var1 = ((((double)pcoff.par_p3 * var1 * var1) / 16384.0) +
             ((double)pcoff.par_p2 * var1)) / 524288.0;
     var1 = (1.0 + (var1 / 32768.0)) * (double)pcoff.par_p1;
    
     double press_comp = 1048576.0 - (double)adc_val.pres_adc;
     press_comp        = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
    
     var1 = ((double)pcoff.par_p9 * press_comp * press_comp) / 2147483648.0;
     var2 = press_comp * ((double)pcoff.par_p8 / 32768.0);
     double var3 = (press_comp / 256.0) * (press_comp / 256.0) *
                   (press_comp / 256.0) * (pcoff.par_p10 / 131072.0);
    
     press_comp  = press_comp + (var1 + var2 + var3 +
                   ((double)pcoff.par_p7 * 128.0)) / 16.0;
     readings->pres_comp = press_comp;

    // NOTE: For Int
    /*
    int32_t var1 = ((int32_t) readings->temp_comp >> 1) - 64000;
    int32_t var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) pcoff.par_p6) >> 2;

    var2 = var2 + ((var1 * (int32_t) pcoff.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t) pcoff.par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t) pcoff.par_p3 << 5)) >> 3)
           + (((int32_t) pcoff.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t) pcoff.par_p1) >> 15;

    int32_t press_comp = 1048576 - adc_val.pres_adc;
    press_comp = (uint32_t) ((adc_val.pres_adc - (var2 >> 12)) * ((uint32_t) 3125));

    if (press_comp >= (1 << 30))
        press_comp = ((press_comp / (uint32_t) var1) << 1);
    else
        press_comp = ((press_comp << 1) / (uint32_t) var1);

    var1 = ((int32_t) pcoff.par_p9 * (int32_t) (((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t) (press_comp >> 2) * (int32_t) pcoff.par_p8) >> 13;

    int32_t var3 = ((int32_t) (press_comp >> 8) * (int32_t) (press_comp >> 8) * (int32_t) (press_comp >> 8)
                    * (int32_t) pcoff.par_p10)
                   >> 17;

    readings->pres_comp
        = (int32_t) (press_comp) + ((var1 + var2 + var3 + ((int32_t) pcoff.par_p7 << 7)) >> 4);
        */
}


void calc_humd (bme680_measurement_t* readings)
{
    // NOTE: For Float
    
     double var1 = adc_val.humd_adc - (((double)hcoff.par_h1 * 16.0) +
                   (((double)hcoff.par_h3 / 2.0) * readings->temp_comp));
     double var2 = var1 * (((double)hcoff.par_h2 / 262144.0) *
                   (1.0 + (((double)hcoff.par_h4 / 16384.0) *
                   readings->temp_comp) + (((double)hcoff.par_h5 / 1048576.0) *
                   readings->temp_comp * readings->temp_comp)));
     double var3 = (double)hcoff.par_h6 / 16384.0;
     double var4 = (double)hcoff.par_h7 / 2097152.0;
     double hum_comp = var2 + ((var3 + (var4 * readings->temp_comp)) * var2 * var2);
     readings->humd_comp = hum_comp;

    // NOTE: For Int
    /*
    int32_t temp_scaled = (int32_t) readings->temp_comp;
    int32_t var1 = (int32_t) adc_val.humd_adc - (int32_t) ((int32_t) hcoff.par_h1 << 4)
                   - (((temp_scaled * (int32_t) hcoff.par_h3) / ((int32_t) 100)) >> 1);
    int32_t var2 = ((int32_t) hcoff.par_h2
                    * (((temp_scaled * (int32_t) hcoff.par_h4) / ((int32_t) 100))
                       + (((temp_scaled * ((temp_scaled * (int32_t) hcoff.par_h5) / ((int32_t) 100))) >> 6)
                          / ((int32_t) 100))
                       + ((int32_t) (1 << 14))))
                   >> 10;
    int32_t var3 = var1 * var2;
    int32_t var4
        = (((int32_t) hcoff.par_h6 << 7) + ((temp_scaled * (int32_t) hcoff.par_h7) / ((int32_t) 100))) >> 4;
    int32_t var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    int32_t var6 = (var4 * var5) >> 1;

    readings->humd_comp = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;
    */
}



void calc_gas (bme680_measurement_t* readings)
{
    // NOTE: For Float
    
     double const_array1[16] = { 1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1 };
     double const_array2[16] = { 8000000, 4000000, 2000000, 1000000, 499500.4995, 248262.1648, 125000,
                                 63004.03226, 31281.28128, 15625, 7812.5, 3906.25, 1953.125, 976.5625,
                                 488.28125, 244.140625 };
     double var1    = (1340.0 + 5.0 * gcoff.range_switching_error) * const_array1[adc_val.gas_range];
     double gas_res = var1 * const_array2[adc_val.gas_range] / (adc_val.gas_adc - 512.0 + var1);
     readings->gas_comp = gas_res;

    // NOTE: For Int
    /*
    int32_t const_array1_int[16]
        = { 2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2126008810, 2147483647, 2130303777,
            2147483647, 2147483647, 2143188679, 2136746228, 2147483647, 2126008810, 2147483647, 2147483647 };
    int32_t const_array2_int[16]
        = { 4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064,
            16016016,   8000000,    4000000,    2000000,   1000000,   500000,    250000,   125000 };

    int64_t var1 = (int64_t) (((1340 + (5 * (int64_t) gcoff.range_switching_error))
                               * ((int64_t) const_array1_int[adc_val.gas_range]))
                              >> 16);
    int64_t var2 = (int64_t) (adc_val.gas_adc << 15) - (int64_t) (1 << 24) + var1;

    readings->gas_comp
        = (int32_t) ((((int64_t) (const_array2_int[adc_val.gas_range] * (int64_t) var1) >> 9) + (var2 >> 1))
                     / var2);
    */
}


void set_reg (uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_data)
{
    uint8_t reg_value = 0;
    // getting the old data
    bme680_i2c_receive (reg_addr, &reg_value, 1);
    bme680_delay_ms (100);

    // setting the new reg value
    reg_value &= ~reg_mask; // clear old values
    reg_value |= reg_data;  // set new values

    // transmitting the new data
    bme680_i2c_transmit (reg_addr, reg_value);
    bme680_delay_ms (100);
}
