#include "app_i2c.h"

int app_i2c_init(i2c_inst_t * i2c, const uint8_t SCL_PIN, const uint8_t SDA_PIN, const bool PUEN) {

    int err = PICO_ERROR_NONE;

    return err;
}

int i2c_reg_read(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * buff, const uint8_t nbytes) {

    int err = PICO_ERROR_NONE;

    err = i2c_write_blocking(i2c, slave_addr, &reg_addr, 1, true);

    if (err != PICO_ERROR_GENERIC) err = i2c_read_blocking(i2c, slave_addr, buff, nbytes, false);

    return err;
}

int i2c_reg_write(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * data, const uint8_t nbytes) {

    int err = PICO_ERROR_NONE;
    uint8_t src[nbytes + 1];
    src[0] = reg_addr;
    src[1] = *data;

    err = i2c_write_blocking(i2c, slave_addr, src, nbytes + 1, false);

    return err;
}
