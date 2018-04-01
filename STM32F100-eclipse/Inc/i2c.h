#ifndef _HAVE_I2C_H_
#define _HAVE_I2C_H_

#include "stm32f1xx_hal.h"




int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
void i2c_set_instance(I2C_HandleTypeDef *hi2c);











#endif // _HAVE_I2C_H_
