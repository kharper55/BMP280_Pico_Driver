#ifndef APP_I2C_H
#define APP_I2C_H

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <string.h> // For memcpy()
#include "util.h"

#define I2C_SPEED_SM     100000  ///< Standard mode speed (kbit/s)
#define I2C_SPEED_FM     400000  ///< Fast mode speed (kbit/s)
#define I2C_SPEED_FMPLUS 1000000 ///< Fm+ speed (kbit/s)
#define I2C_SPEED_HS     3400000 ///< High speed (kbit/s) 
#define I2C_SPEED_UFM    5000000 ///< Ultra fast mode speed (kbit/s)

#define I2C_BUFF_SIZE_MAX 32

void app_i2c_init(i2c_inst_t * i2c, const uint8_t scl, const uint8_t sda, const uint32_t fclk, const bool puen);
pico_err_t i2c_reg_read(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * buff, const size_t nbytes);
pico_err_t i2c_reg_write(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, const uint8_t * data, const size_t nbytes);

#endif // APP_I2C_H