#ifndef _HAVE_MOTOR_H_
#define _HAVE_MOTOR_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"


typedef struct {
	GPIO_TypeDef 			*A_port;
	GPIO_TypeDef 			*B_port;
	uint16_t				A_pin;
	uint16_t				B_pin;
	TIM_HandleTypeDef* 		p_timer_pwm;
	uint32_t				timer_channel;
}motor_t;


void motor_stop(motor_t *p_motor);
void motor_go(motor_t *p_motor, int dir, float speed_norm);
void motor_init(motor_t *p_motor, GPIO_TypeDef *A_port, uint16_t A_pin, GPIO_TypeDef *B_port, uint16_t B_pin, TIM_HandleTypeDef *p_timer_pwm, uint32_t timer_channel);











#endif // HAVE_MOTOR_H
