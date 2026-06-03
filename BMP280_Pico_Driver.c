/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


// This code demonstrates assigning stdio to RP2040's USB peripheral for serial comms
// as well as communicates with the BMP280 enivornmental sensor via SPI or I2C

// Standard includes
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <math.h>

// Custom includes
#include "bmx280.h"
#include "app_i2c.h"
#include "util.h"

#define SCL_PIN 16
#define SDA_PIN 17

int main() {

    stdio_init_all();

    app_i2c_init(i2c0, SCL_PIN, SDA_PIN, I2C_SPEED_FM, false);
    //bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C)); // No clue what this line does

    //int32_t temp_raw;
    //int32_t press_raw;
    //int32_t hum_raw;
    int32_t temp;
    uint32_t press;
    uint32_t hum;
    bmx280_osrs_t temp_osrs = BMX280_OVERSAMP_X1;
    bmx280_osrs_t press_osrs = BMX280_SKIP_MEAS;
    bmx280_osrs_t hum_osrs = press_osrs;

    const extern bmx280_config_t bmx280_indoor_nav_cfg;

    //bmx280_config_t myCfg = {BMX280_PWR_MODE_NORM, BMX280_TSTDBY_0_5MS, BMX280_FILT_OFF, temp_osrs, press_osrs, hum_osrs};

    bmx280_config_t myCfg = bmx280_indoor_nav_cfg;

    bmx280_init(&myCfg, false);

    sleep_ms(100);

    while (true) {

        /*printf("Reading pressure...\n\n");
        i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_PRESS_MSB, rxdata, 3); // Read back pressure measurement, let the BMP280 auto increment registers
        press_raw = BMP280_PACK_DATA(rxdata[0], rxdata[1], rxdata[2]);

        printf("Reading temperature...\n\n");
        i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMX280_REG_TEMP_MSB, rxdata, 3); // Read back temperature measurement
        temp_raw = BMP280_PACK_DATA(rxdata[0], rxdata[1], rxdata[2]);

        printf("Reading humidity...\n\n");
        i2c_reg_read(i2c0, BMX280_SLAVE_ADDR, BMP280_REG_HUM_MSB, rxdata, 2); // Read back humidity measurement
        hum_raw = (int32_t)(rxdata[0] << 8 | rxdata[1]);

        temp = bmx280_compensate_T_int32(temp_raw);
        press = bmx280_compensate_P_int64(press_raw);
        hum = bme280_compensate_H_int32(hum_raw);*/

        bmx280_read_temp(&temp);

        if (myCfg.osrs_press != BMX280_SKIP_MEAS && myCfg.osrs_hum != BMX280_SKIP_MEAS) {
            bmx280_read_press(&press);
            bme280_read_hum(&hum);

            //printf("Raw Pressure: %d Raw Temperature: %d Raw Humidity: %d\n", press_raw, temp_raw, hum_raw);
            printf("Pressure (Pa): %.2f Temperature (°F): %.2f Humidity (%%): %.2f\n", press/256.0, C_2_F(temp/100.0), hum/1024.0);
        }

        else printf("Temperature (°F): %.2f\n", C_2_F(temp/100.0));
            
        sleep_ms(10);
    }
}
