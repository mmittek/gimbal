#include "i2c.h"
#include <string.h>


I2C_HandleTypeDef *i2c_instance = NULL;


void i2c_set_instance(I2C_HandleTypeDef *hi2c) {
	i2c_instance = hi2c;
}


// Using code from @see: https://jure.tuta.si/?p=7
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    unsigned char new_data[length + 1];
    memcpy(new_data + 1, data, length);
    new_data[0] = reg_addr;

    if (HAL_I2C_Master_Transmit(i2c_instance, (uint16_t)slave_addr<<1, new_data, length+1, 1000) == HAL_OK) {
		while (HAL_I2C_GetState(i2c_instance) != HAL_I2C_STATE_READY){}
		return 0;
    }

    return -1;
}


int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {

	// Write register address first
    if (HAL_I2C_Master_Transmit(i2c_instance, (uint16_t)slave_addr<<1, &reg_addr, 1, 1000) == HAL_OK) {
		while (HAL_I2C_GetState(i2c_instance) != HAL_I2C_STATE_READY){}
		if(HAL_I2C_Master_Receive(i2c_instance, (uint16_t)slave_addr<<1, data, length, 1000) == HAL_OK) {
			while (HAL_I2C_GetState(i2c_instance) != HAL_I2C_STATE_READY){}
			return 0;
		}
    }

    return -1;
}
