#include "pico/stdlib.h"
#include <stdio.h>
#include "app_i2c.h"

// Note: BME280 contains humidity sensor. BMP280 does not.
// BMX stands for either BME or BMP - these values are valid for either device
// Where a feature is only relevant for one device or the other, the register is referred to as BME280 or BMP280

// BME280: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
// BMP280: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

typedef int32_t BMX280_S32_t;
typedef uint32_t BMX280_U32_t;
typedef long long int BMX280_S64_t;

#define BMX280_I2C true // True for I2C, false for SPI
#define BMX280_SPI !BMX280_I2C
#define BMX280_I2C_PORT i2c0
//#define BMX280_SPI_PORT spi0

#define BMX280_SLAVE_ADDR 0x76 // Value dependent on SDO voltage. If SDO is '1', use 0x77
#define BME280_SLAVE_ADDR 0x68

// Contains the chip identification number chip_id[7:0], which is 0x58 for BMP280 and
// 0x60 for BME280. This number can be read as soon as the device finished the power-on-reset.
#define BMX280_REG_ID         0xD0 // Read only
#define BMP280_REG_ID_VAL     0x58
#define BME280_REG_ID_VAL     0x60

// Contains the soft reset word reset[7:0]. If the value 0xB6 is written to the register,
// the device is reset using the complete power-on-reset procedure. Writing other values than 0xB6 has
// no effect. The readout value is always 0x00.
#define BMX280_REG_RESET       0xE0 // Write only
#define BMX280_REG_RESET_VALUE 0xB6

#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS_HBIT 0 // Ensure only writing 3 bit data

// Contains two bits which indicate the status of the device. Bit 3 is automatically set to '1'
// whenever a conversion is runnign and back to '0' when results have been transferred to the data
// registers. Bit 0 is automatically set to '1'  when the NVM data are being copied to image 
// registers and back to ‘0’ when the copying is done. The data are copied at power-on-reset 
// and before every conversion.
#define BMX280_REG_STATUS     0xF3 // Read only
#define BMX280_REG_STATUS_MEASBIT 3 
#define BMX280_REG_STATUS_IMGBIT 0

// Sets the data acquisition options of the device. Bits [7:5] controls oversampling for temperature.
// Bits [4:2] controls oversampling for pressure. Bits [1:0] control the power mode of the device.
#define BMX280_REG_CTRL_MEAS  0xF4 // R/W
// Oversampling settings (bits [7:5] and [4:2])
#define BMX280_REG_CTRL_MEAS_TBIT 7-3+1 // Ensure only writing 3 bit data
#define BMX280_REG_CTRL_MEAS_PBIT 4-3+1 // Ensure only writing 3 bit data
#define BMX280_SKIP_MEAS      0b000
#define BMX280_OVERSAMP_X1    0b001 // 1x oversampling 
#define BMX280_OVERSAMP_X2    0b010 // 2x oversampling 
#define BMX280_OVERSAMP_X4    0b011 // 4x oversampling 
#define BMX280_OVERSAMP_X8    0b100 // 8x oversampling 
#define BMX280_OVERSAMP_X16   0b101 // 16x oversampling
// Power settings (bits [1:0])
#define BMX280_REG_CTRL_MEAS_PWR 1-2+1  // Ensure only writing 2 bit data
#define BMX280_PWR_MODE_SLP   0b00  // Sleep mode
#define BMX280_PWR_MODE_FRC   0b01  // Forced mode
#define BMX280_PWR_MODE_NORM  0b11  // Normal mode

// Sets the rate, filter and interface options of the device. Writes to the “config”
// register in normal mode may be ignored. In sleep mode writes are not ignored.
// Bits [7:5] set the inactive duration / tstandby in normal mode.
// Bits [4:2] control the time constant of the IIR filter
// Bit 0 enables 3-wire SPI interface when set to '1'
#define BMX280_REG_CONFIG     0xF5 // R/W 
// Tstdby settings
#define BMX280_REG_CONFIG_TSTDBY_BIT 7
#define BMX280_TSTDBY_0_5ms  0b000
#define BMX280_TSTDBY_62_5ms 0b001
#define BMX280_TSTDBY_125ms  0b010
#define BMX280_TSTDBY_250ms  0b011
#define BMX280_TSTDBY_500ms  0b100
#define BMX280_TSTDBY_1000ms 0b101
#define BMP280_TSTDBY_10ms 0b110 // 2000ms for BMP280
#define BMP280_TSTDBY_20ms 0b111 // 4000ms for BMP280
#define BME280_TSTDBY_2000ms BMP280_TSTDBY_10ms
#define BME280_TSTDBY_4000ms BMP280_TSTDBY_20ms

// Filter settings
#define BMX280_REG_CONFIG_FILT_BIT 4
#define BMX280_FILT_COEFF_OFF 0b000 
#define BMX280_FILT_COEFF_2 0b001
#define BMX280_FILT_COEFF_4 0b010
#define BMX280_FILT_COEFF_8 0b011
#define BMX280_FILT_COEFF_16 0b100 // And 0b101, 0b110, 0b111

// Contains the raw pressure measurement output data up[19:0]
#define BMX280_REG_PRESS_MSB  0xF7 // Read only, bits [19:12] of pressure data
#define BMX280_REG_PRESS_LSB  0xF8 // Read only, bits [11:4] of pressure data
#define BMX280_REG_PRESS_XLSB 0xF9 // Read only, bits [3:0] of pressure data exist in bits [7:4] of this register
#define BMX280_PRESS_RAW_LEN 3

// Contains the raw temperature measurement output data ut[19:0]
#define BMX280_REG_TEMP_MSB   0xFA // Read only, bits [19:12] of temperature data
#define BMX280_REG_TEMP_LSB   0xFB // Read only, bits [11:4] of temperature data
#define BMX280_REG_TEMP_XLSB  0xFC // Read only, bits [3:0] of temperature data exist in bits [7:4] of this register
#define BMX280_TEMP_RAW_LEN  3

// Contains the raw humidity measurement output data uh[15:0]
#define BME280_REG_HUM_MSB 0xFD // Read only, bits [15:8] of humidity data
#define BME280_REG_HUM_LSB 0xFE // Read only, bits [7:0] of humidity data
#define BME280_HUM_RAW_LEN   2

/*
Temperature measurement can be enabled or skipped. Skipping the measurement could be useful to
measure pressure extremely rapidly. When enabled, several oversampling options exist. Each
oversampling step reduces noise and increases the output resolution by one bit, which is stored in the
XLSB data register 0xFC. Enabling/disabling the temperature measurement and oversampling setting
are selected through the osrs_t[2:0] bits in control register 0xF4.
*/

/*
The trimming parameters are programmed into the devices’ non-volatile memory (NVM) during
production and cannot be altered by the customer. Each compensation word is a 16-bit signed or
unsigned integer value stored in two’s complement. As the memory is organized into 8-bit words, two
words must always be combined in order to represent the compensation word. The 8-bit registers are
named calib00…calib25 and are stored at memory addresses 0x88…0xA1. The corresponding
compensation words are named dig_T# for temperature compensation related values and dig_P# for
pressure compensation related values. The mapping is shown in Table 17.
*/
#define BMX280_REG_CALIB00 0x88
#define BMX280_REG_CALIB01 0x89 
#define BMX280_REG_CALIB02 0x8A
#define BMX280_REG_CALIB03 0x8B
#define BMX280_REG_CALIB04 0x8C
#define BMX280_REG_CALIB05 0x8D
#define BMX280_REG_CALIB06 0x8E
#define BMX280_REG_CALIB07 0x8F
#define BMX280_REG_CALIB08 0x90
#define BMX280_REG_CALIB09 0x91
#define BMX280_REG_CALIB10 0x92
#define BMX280_REG_CALIB11 0x93
#define BMX280_REG_CALIB12 0x94
#define BMX280_REG_CALIB13 0x95
#define BMX280_REG_CALIB14 0x96
#define BMX280_REG_CALIB15 0x97
#define BMX280_REG_CALIB16 0x98
#define BMX280_REG_CALIB17 0x99
#define BMX280_REG_CALIB18 0x9A
#define BMX280_REG_CALIB19 0x9B
#define BMX280_REG_CALIB20 0x9C
#define BMX280_REG_CALIB21 0x9D
#define BMX280_REG_CALIB22 0x9E
#define BMX280_REG_CALIB23 0x9F
#define BMX280_REG_CALIB24 0xA0 // Unused / reserved 
#define BMX280_REG_CALIB25 0xA1 // Unused / reserved in BMX280
#define BMX280_REG_CALIB26 0xE1
#define BMX280_REG_CALIB27 0xE2
#define BMX280_REG_CALIB28 0xE3
#define BMX280_REG_CALIB29 0xE4
#define BMX280_REG_CALIB30 0xE5
#define BMX280_REG_CALIB31 0xE6
#define BMX280_REG_CALIB32 0xE7
#define BMX280_REG_CALIB33 0xE8
#define BMX280_REG_CALIB34 0xE9
#define BMX280_REG_CALIB35 0xEA
#define BMX280_REG_CALIB36 0xEB
#define BMX280_REG_CALIB37 0xEC
#define BMX280_REG_CALIB38 0xED
#define BMX280_REG_CALIB39 0xEE
#define BMX280_REG_CALIB40 0xEF
#define BMX280_REG_CALIB41 0xF0

// Compensation parameter storage (temperature), 16 bit words stored in adjacent 8 bit registers, LSB at lower addr
#define BMX280_REG_DIG_T1 BMX280_REG_CALIB00 // Unsigned short
#define BMX280_REG_DIG_T2 BMX280_REG_CALIB02 // Signed short
#define BMX280_REG_DIG_T3 BMX280_REG_CALIB04 // Signed short

// Compensation parameter storage (pressure), 16 bit words stored in adjacent 8 bit registers, LSB at lower addr
#define BMX280_REG_DIG_P1 BMX280_REG_CALIB06 // Unsigned short
#define BMX280_REG_DIG_P2 BMX280_REG_CALIB08 // Signed short
#define BMX280_REG_DIG_P3 BMX280_REG_CALIB10 // Signed short
#define BMX280_REG_DIG_P4 BMX280_REG_CALIB12 // Signed short
#define BMX280_REG_DIG_P5 BMX280_REG_CALIB14 // Signed short
#define BMX280_REG_DIG_P6 BMX280_REG_CALIB16 // Signed short
#define BMX280_REG_DIG_P7 BMX280_REG_CALIB18 // Signed short
#define BMX280_REG_DIG_P8 BMX280_REG_CALIB20 // Signed short
#define BMX280_REG_DIG_P9 BMX280_REG_CALIB22 // Signed short

// Compensation parameter storage (humidity), 16 bit words stored in adjacent 8 bit registers, LSB at lower addr
#define BME280_REG_DIG_H1 BMX280_REG_CALIB25 // Unsigned char
#define BME280_REG_DIG_H2 BMX280_REG_CALIB26 // Signed short
#define BME280_REG_DIG_H3 BMX280_REG_CALIB28 // Unsigned char
#define BME280_REG_DIG_H4 BMX280_REG_CALIB29 // Signed short (11 bits, use only bits [3:0] from 0xE5)
#define BME280_REG_DIG_H5 BMX280_REG_CALIB30 // Signed short (11 bits, use only bits [7:4] from 0xE5)
#define BME280_REG_DIG_H6 BMX280_REG_CALIB32 // Signed char

// Pack the MSB, LSB, and XLSB register values into a 32 bit signed int
#define BMP280_PACK_DATA(MSB, LSB, XLSB) (int32_t)(MSB << 12 | LSB << 4 | (XLSB >> 4))
#define PASCAL_2_ATM(pascal) (double)(pascal * 9.86923 * 1/1000000) // Use to convert pressure values from BMP280 to units of ATM
#define C_2_F(c) (double)(c * 9/5 + 32)

typedef enum {
    BMX280_BMP280,
    BMX280_BME280
} bmx280_type_t;

// Compensation functions provided by Bosch in BMX280 datasheet

// Returns temperature in DegC as signed 32 bit integer, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value. Must call bmp280_compensate_T_int32() before pressure/humidity compensation since 
// these depend on t_fine, which is updated via bmp280_compensate_T_int32()
BMX280_S32_t bmx280_compensate_T_int32(BMX280_S32_t adc_T);

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMX280_U32_t bmx280_compensate_P_int64(BMX280_S32_t adc_P);

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BMX280_U32_t bme280_compensate_H_int32(BMX280_S32_t adc_H);

int bmx280_init(bool rst); // need to update this with settings for run-time config...
int bmx280_sw_reset(void);
int bmx280_read_temp(int32_t * temp);
int bmx280_read_press(uint32_t * press);
int bme280_read_hum(uint32_t * hum);