#include "bmx280.h"

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

BMX280_S32_t t_fine;

// Bosch recommended configuration for weather monitoring application
const bmx280_config_t bmx280_weather_mon_cfg = { // 0.16uA current consumption
    BMX280_PWR_MODE_FRC,
    BMX280_TSTDBY_0_5MS, // Value inconsequential in forced operating mode
    BMX280_FILT_OFF,
    BMX280_OVERSAMP_X1,  // Temp
    BMX280_OVERSAMP_X1,  // Press
    BMX280_OVERSAMP_X1   // Hum
};

// Bosch recommended configuration for humidity sensing application
const bmx280_config_t bmx280_hum_sensing_cfg = { // 2.9uA current consumption
    BMX280_PWR_MODE_FRC,
    BMX280_TSTDBY_0_5MS, // Value inconsequential in forced operating mode
    BMX280_FILT_OFF,
    BMX280_OVERSAMP_X1,  // Temp
    BMX280_SKIP_MEAS,    // Press
    BMX280_OVERSAMP_X1   // Hum
};

// Bosch recommended configuration for indoor navigation application
const bmx280_config_t bmx280_indoor_nav_cfg = { // 633uA current consumption
    BMX280_PWR_MODE_NORM,
    BMX280_TSTDBY_0_5MS,
    BMX280_FILT_COEFF_16,
    BMX280_OVERSAMP_X2,  // Temp
    BMX280_OVERSAMP_X16, // Press
    BMX280_OVERSAMP_X1   // Hum
};

// Bosch recommended configuration for gaming application
const bmx280_config_t bmx280_gaming_cfg = { // 581uA current consumption
    BMX280_PWR_MODE_NORM,
    BMX280_TSTDBY_0_5MS,
    BMX280_FILT_COEFF_16,
    BMX280_OVERSAMP_X1,  // Temp
    BMX280_OVERSAMP_X4,  // Press
    BMX280_SKIP_MEAS     // Hum
};

// Compensation functions provided by Bosch in BMX280 datasheet

// Returns temperature in DegC as signed 32 bit integer, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value. Must call bmp280_compensate_T_int32() before pressure/humidity compensation since 
// these depend on t_fine, which is updated via bmp280_compensate_T_int32()
BMX280_S32_t bmx280_compensate_T_int32(BMX280_S32_t adc_T) {
    BMX280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BMX280_S32_t)dig_T1<<1))) * ((BMX280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMX280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMX280_S32_t)dig_T1)))>> 12) * ((BMX280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMX280_U32_t bmx280_compensate_P_int64(BMX280_S32_t adc_P) {
    BMX280_S64_t var1, var2, p;
    var1 = ((BMX280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BMX280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BMX280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BMX280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BMX280_S64_t)dig_P3)>>8) + ((var1 * (BMX280_S64_t)dig_P2)<<12);
    var1 = (((((BMX280_S64_t)1)<<47)+var1))*((BMX280_S64_t)dig_P1)>>33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BMX280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BMX280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BMX280_S64_t)dig_P7)<<4);
    return (BMX280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BMX280_U32_t bme280_compensate_H_int32(BMX280_S32_t adc_H) {
    BMX280_S32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((BMX280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BMX280_S32_t)dig_H4) << 20) - (((BMX280_S32_t)dig_H5) *
    v_x1_u32r)) + ((BMX280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((BMX280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BMX280_S32_t)dig_H3)) >> 11) +
    ((BMX280_S32_t)32768))) >> 10) + ((BMX280_S32_t)2097152)) * ((BMX280_S32_t)dig_H2) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((BMX280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (BMX280_U32_t)(v_x1_u32r>>12);
}

// ===========================================================================================================================================
pico_err_t bmx280_sw_reset(void) {

    uint8_t txdata = BMX280_REG_RESET_VALUE;

    pico_err_t err = i2c_reg_write(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_RESET, &txdata, 1); // Force software reset by writing the reset word to device
    
    return err;
}

// ===========================================================================================================================================
pico_err_t bmx280_init(bmx280_config_t * cfg, bool rst) { // need to update this with settings for run-time config...

    bmx280_mode_t mode = cfg->mode;
    bmx280_tsdby_t tsdby = cfg->tsdby;
    bmx280_filter_t filt = cfg->filt;
    bmx280_osrs_t osrs_temp = cfg->osrs_temp;
    bmx280_osrs_t osrs_press = cfg->osrs_press;
    bmx280_osrs_t osrs_hum = cfg->osrs_hum;

    uint8_t rxData[2];

    uint8_t txdata = (mode << BMX280_REG_CTRL_MEAS_PWR) & 0xFF; // Set power mode to normal
    txdata |= (osrs_temp << BMX280_REG_CTRL_MEAS_TBIT); // Set oversampling value for temperature to x1 (enable its measurement)
    txdata |= (osrs_press << BMX280_REG_CTRL_MEAS_PBIT); // Set oversampling value for pressure to x1 (enable its measurement)
    
    uint8_t txdata2 = osrs_hum << BME280_REG_CTRL_MEAS_HBIT; // Set oversampling value for humidity to x1 (enable its measurement)

    uint8_t device_id = 0x00;
    uint8_t * device_id_str;

    uint8_t config_reg_data = (tsdby << BMX280_REG_CONFIG_TSTDBY_BIT) | (filt << BMX280_REG_CONFIG_FILT_BIT); // In sleep mode, writes to config register are ignored.

    pico_err_t err = PICO_ERROR_NONE;

    //sleep_ms(6000);

    printf("Reading BMX280 ID Register @ Address 0xD0...\n\n");

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_ID, &device_id, 1); 
    if (err == PICO_ERROR_GENERIC) return err;

    device_id_str = (device_id == BMP280_REG_ID_VAL ? "BMP280" : (device_id == BME280_REG_ID_VAL ? "BME280" : "UNKNOWN"));
    
    printf("Device ID is 0x%X. Device is %s.\n\n", device_id, device_id_str);
    if (device_id != BMP280_REG_ID_VAL && device_id != BME280_REG_ID_VAL) {
        err = PICO_ERROR_INVALID_ADDRESS;
        return err;
    }

    if (rst) {
        err = bmx280_sw_reset();
        if (err == PICO_ERROR_GENERIC) return err;
        sleep_ms(10);
    }

    printf("Fetching compensation parameters...\n\n");

    // Temp compensation values

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_T1, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_T1 = (uint16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_T1: %u\n", dig_T1);
    
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_T2, rxData, 2);
    if (err == PICO_ERROR_GENERIC) return err;
    dig_T2 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_T2: %d\n", dig_T2);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_T3, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_T3 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_T3: %d\n", dig_T3);

    // Pressure compensation values

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P1, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P1 = (uint16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P1: %u\n", dig_P1);
    
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P2, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P2 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P2: %d\n", dig_P2);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P3, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P3 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P3: %d\n", dig_P3);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P4, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P4 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P4: %d\n", dig_P4);
    
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P5, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P5 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P5: %d\n", dig_P5);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P6, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P6 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P6: %d\n", dig_P6);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P7, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P7 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P7: %d\n", dig_P7);
    
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P8, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P8 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P8: %d\n", dig_P8);

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_DIG_P9, rxData, 2); 
    if (err == PICO_ERROR_GENERIC) return err;
    dig_P9 = (int16_t)(rxData[0] | (rxData[1] << 8));
    printf("dig_P9: %d\n\n", dig_P9);

    // Humidity compensation values

    if (device_id == BME280_REG_ID_VAL) { // Only BME280 has the humidity peripheral

        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H1, rxData, 1); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H1 = (uint8_t)rxData[0];
        printf("dig_H1: %d\n", dig_H1);
        
        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H2, rxData, 2); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H2 = (int16_t)(rxData[0] | (rxData[1] << 8));
        printf("dig_H2: %d\n", dig_H2);

        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H3, rxData, 2); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H3 = (uint8_t)rxData[0];
        printf("dig_H3: %d\n", dig_H3);

        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H4, rxData, 2); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H4 = (int16_t)((rxData[0] << 4) | (rxData[1] & 0x0F)); // Should double check this
        printf("dig_H4: %d\n", dig_H4);
        
        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H5, rxData, 2); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H5 = (int16_t)((rxData[0] >> 4) | (rxData[1] << 4)); // Should double check this
        printf("dig_H5: %d\n", dig_H5);

        err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_DIG_H6, rxData, 1); 
        if (err == PICO_ERROR_GENERIC) return err;
        dig_H6 = (int8_t)rxData[0];
        printf("dig_H6: %d\n\n", dig_H6);
    }

    printf("Configuring %s for normal mode operation with no oversampling...\n\n", device_id_str);

    // Note: The “ctrl_meas” register sets the pressure and temperature data acquisition options of the device. The
    // register needs to be written after changing “ctrl_hum” for the changes to become effective.

    // But keep in mind that the “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to this
    // register only become effective after a write operation to “ctrl_meas”.

    err = i2c_reg_write(i2c0, BMX280_SLAVE_ADDR, BME280_REG_CTRL_HUM, &txdata2, 1); // Enable humidity by setting oversampling to x1
    if (err == PICO_ERROR_GENERIC) return err;

    err = i2c_reg_write(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_CTRL_MEAS, &txdata, 1); // Take out of sleep and enable temp / pressure
    if (err == PICO_ERROR_GENERIC) return err;
    
    printf("Done configuring %s.\n\n", device_id_str);

    return err;

}

pico_err_t bmx280_status(bmx280_status_t * status) {
    pico_err_t err = PICO_ERROR_NONE;
    uint8_t buff;

    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_STATUS, &buff, 1); // Read back status register
    if (err == 1) *status = (((buff >> BMX280_REG_STATUS_MEASBIT) & 0x01) | (((buff >> BMX280_REG_STATUS_IMGBIT) & 0x01) << 1));
    
    return err;
}

// ===========================================================================================================================================
pico_err_t bmx280_self_test(bmx280_self_test_result_t * result) {
    return 0;
}

/* NEED TO LOOK INTO THE FOLLOWING...
- Whats the deal with BME280 vs BMP280 data formatting... confused about XLSB usage on BME280...? Its different than from BMP280
- Need to think about how to handle humidity / pressure measurement without tfine? If temperature is disabled, can we grab these values?...
Not important for the application but matters for the driver... Should probably not go this route, requires a lot of application knowledge to
make such a call... with slow changing temperature, its prob fine...
- Add Doxygen style comments
- consider making the raw functions return only: PICO_ERROR_NONE instead of byte counts
- play with various filter and tsdby settings, etc.. investigate best low power settings
*/

// ===========================================================================================================================================
static pico_err_t bmx280_read_temp_raw(int32_t * temp_raw) {

    pico_err_t err = PICO_ERROR_NONE;
    uint8_t buff[BMX280_TEMP_RAW_LEN];

    //printf("Reading temperature...\n\n");
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_TEMP_MSB, buff, BMX280_TEMP_RAW_LEN); // Read back temperature measurement
    if (err != PICO_ERROR_GENERIC && err == BMX280_TEMP_RAW_LEN) *temp_raw = BMX280_PACK_DATA_20BIT(buff[0], buff[1], buff[2]);
    
    return err;
}

// ===========================================================================================================================================
pico_err_t bmx280_read_temp(int32_t * temp) {

    pico_err_t err = PICO_ERROR_NONE;
    int32_t temp_raw;

    err = bmx280_read_temp_raw(&temp_raw);
    if (err == BMX280_TEMP_RAW_LEN) *temp = bmx280_compensate_T_int32(temp_raw);

    return err;
}

// ===========================================================================================================================================
static pico_err_t bmx280_read_press_raw(int32_t * press_raw) {

    pico_err_t err = PICO_ERROR_NONE;
    uint8_t buff[BMX280_PRESS_RAW_LEN];

    //printf("Reading pressure...\n\n");
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_PRESS_MSB, buff, BMX280_PRESS_RAW_LEN); // Read back pressure measurement, let the BMP280 auto increment registers
    if (err != PICO_ERROR_GENERIC && err == BMX280_PRESS_RAW_LEN) *press_raw = BMX280_PACK_DATA_20BIT(buff[0], buff[1], buff[2]);

    return err;
}

// ===========================================================================================================================================
pico_err_t bmx280_read_press(uint32_t * press) {

    pico_err_t err = PICO_ERROR_NONE;
    int32_t press_raw;

    err = bmx280_read_press_raw(&press_raw);
    if (err == BMX280_PRESS_RAW_LEN) *press = bmx280_compensate_P_int64(press_raw);

    return err;
}

// ===========================================================================================================================================
static pico_err_t bme280_read_hum_raw(int32_t * hum_raw) {

    pico_err_t err = PICO_ERROR_NONE;
    uint8_t buff[BME280_HUM_RAW_LEN];

    //printf("Reading humidity...\n\n");
    err = i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BME280_REG_HUM_MSB, buff, BME280_HUM_RAW_LEN); // Read back humidity measurement
    if (err != PICO_ERROR_GENERIC && err == BME280_HUM_RAW_LEN) *hum_raw = BME280_PACK_DATA_16BIT(buff[0], buff[1]);

    return err;
}

// ===========================================================================================================================================
pico_err_t bme280_read_hum(uint32_t * hum) {

    pico_err_t err = PICO_ERROR_NONE;
    int32_t hum_raw;

    err = bme280_read_hum_raw(&hum_raw);
    if (err == BME280_HUM_RAW_LEN) *hum = bme280_compensate_H_int32(hum_raw);

    return err;
}
