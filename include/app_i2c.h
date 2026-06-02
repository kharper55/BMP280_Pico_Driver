#include "hardware/i2c.h"

int app_i2c_init(i2c_inst_t * i2c, const uint8_t SCL_PIN, const uint8_t SDA_PIN, const bool PUEN); // Still need to write this
int i2c_reg_read(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * buff, const uint8_t nbytes);
int i2c_reg_write(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * data, const uint8_t nbytes);