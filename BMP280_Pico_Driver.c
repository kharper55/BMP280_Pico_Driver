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

int main() {
    stdio_init_all();

    i2c_init(i2c0, 400 * 1000); // 400kHz

    gpio_set_function(/*PICO_DEFAULT_I2C_SDA_PIN*/17, GPIO_FUNC_I2C);
    gpio_set_function(/*PICO_DEFAULT_I2C_SCL_PIN*/16, GPIO_FUNC_I2C);
    //gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    //gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN); // Dev kits in use feature pullups already
    bi_decl(bi_2pins_with_func(17, 16, GPIO_FUNC_I2C));

    uint8_t rxdata[3];

    //int32_t temp_raw;
    //int32_t press_raw;
    //int32_t hum_raw;
    int32_t temp;
    uint32_t press;
    uint32_t hum;

    bmx280_init(false);

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
        bmx280_read_press(&press);
        bme280_read_hum(&hum);

        // Should investigate the bit resolution for various BME280 settings... Not always 20 bit depending on filter and oversampling settings

        //printf("Raw Pressure: %d Raw Temperature: %d Raw Humidity: %d\n", press_raw, temp_raw, hum_raw);
        printf("Pressure (Pa): %.2f Temperature (°F): %.2f Humidity (%%): %.2f\n\n", press/256.0, C_2_F(temp/100.0), hum/1024.0);

        sleep_ms(1000);
    }
}
