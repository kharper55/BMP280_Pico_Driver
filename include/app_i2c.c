#include "app_i2c.h"

/**
 * Initialize an I2C peripheral for use as a master/controller.
 * Note that the specified SDA and SCL pins must support the selected I2C
 * peripheral instance and no error checking is performed to ensure this.
 * @param[in]  i2c        I2C peripheral instance.
 * @param[in]  scl        Pin number for SCL pin.
 * @param[in]  sda        Pin number for SDA pin.
 * @param[out] fclk       Bit rate for I2C transmissions. 
 * @param[in]  puen       Enable internal pullups.
 * 
 * @return None
 */   
void app_i2c_init(i2c_inst_t * i2c, const uint8_t scl, const uint8_t sda, const uint32_t fclk, const bool puen) {

    int err = PICO_ERROR_NONE;

    i2c_init(i2c, fclk); // 400kHz

    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);

    if (puen) {
        gpio_pull_up(sda);
        gpio_pull_up(scl); 
    }
}

/**
 * Read a number of registers from an I2C slave.
 * @param[in]  i2c        I2C peripheral instance.
 * @param[in]  slave_addr 7-bit I2C slave address.
 * @param[in]  reg_addr   Register address to begin reading from.
 * @param[out] buff       Destination buffer for received data. Must be at least as large as nbytes.
 * @param[in]  nbytes     Number of bytes to read.
 * 
 * @return Number of bytes read on success, PICO_ERROR_GENERIC on failure.
 */  
pico_err_t i2c_reg_read(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, uint8_t * buff, const size_t nbytes) {

    int err = PICO_ERROR_NONE;

    err = i2c_write_blocking(i2c, slave_addr, &reg_addr, 1, true);

    if (err != PICO_ERROR_GENERIC) err = i2c_read_blocking(i2c, slave_addr, buff, nbytes, false);

    return err;
}

/**
 * Write to a number of registers on an I2C slave.
 * @param[in]  i2c        I2C peripheral instance.
 * @param[in]  slave_addr 7-bit I2C slave address.
 * @param[in]  reg_addr   Register address to begin reading from.
 * @param[in]  data       Data to transmit. Must be at least as large as nbytes.
 * @param[in]  nbytes     Number of bytes to write.
 * 
 * @return Number of bytes written on success, PICO_ERROR_GENERIC on failure.
 */  
pico_err_t i2c_reg_write(i2c_inst_t * i2c, const uint8_t slave_addr, const uint8_t reg_addr, const uint8_t * data, const size_t nbytes) {

    int err = PICO_ERROR_NONE;
    
    if (nbytes > I2C_BUFF_SIZE_MAX || nbytes == 0) return PICO_ERROR_GENERIC;

    uint8_t src[nbytes + 1]; // VLA chosen to minimize stack burden

    src[0] = reg_addr;
    memcpy(&src[1], data, nbytes);

    err = i2c_write_blocking(i2c, slave_addr, src, nbytes + 1, false);
    
    return err;
}
