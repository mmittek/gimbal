#include "motor.h"
#include <math.h>

void motor_set_pins(motor_t *p_motor, uint8_t A, uint8_t B) {
	HAL_GPIO_WritePin(p_motor->A_port, p_motor->A_pin, A);
	HAL_GPIO_WritePin(p_motor->B_port, p_motor->B_pin, B);
}

void motor_stop(motor_t *p_motor) {
	HAL_TIM_PWM_Stop(p_motor->p_timer_pwm, p_motor->timer_channel);
	motor_set_pins(p_motor, GPIO_PIN_RESET,GPIO_PIN_RESET);
}


HAL_StatusTypeDef motor_set_pwm_config(motor_t *p_motor, float duty_cycle) {
	if(duty_cycle < 0) duty_cycle = 0;
	if(duty_cycle > 1) duty_cycle = 1;

// Take this configuration from main
	TIM_OC_InitTypeDef sConfigOC;
	  sConfigOC.OCMode = TIM_OCMODE_PWM2;
	  sConfigOC.Pulse = (float)p_motor->p_timer_pwm->Init.Period*(1.0-duty_cycle);			// how long is the low
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  return HAL_TIM_PWM_ConfigChannel(p_motor->p_timer_pwm, &sConfigOC, p_motor->timer_channel);
}


float err_sum = 0.0f;
float prev_err = 0.0f;

void motor_go(motor_t *p_motor, int dir, float speed_norm) {
	if(dir == 0 || speed_norm == 0.0f) {
		motor_stop(p_motor);
		return;
	}

	if(dir > 0) {
		motor_set_pins(p_motor, GPIO_PIN_RESET, GPIO_PIN_SET);
	}else if(dir < 0) {
		motor_set_pins(p_motor, GPIO_PIN_SET, GPIO_PIN_RESET);
	}
	motor_set_pwm_config(p_motor, speed_norm);
	HAL_TIM_PWM_Start(p_motor->p_timer_pwm, p_motor->timer_channel);
}


void motor_init(motor_t *p_motor, GPIO_TypeDef *A_port, uint16_t A_pin, GPIO_TypeDef *B_port, uint16_t B_pin, TIM_HandleTypeDef *p_timer_pwm, uint32_t timer_channel) {
	p_motor->A_port = A_port;
	p_motor->B_port = B_port;
	p_motor->A_pin = A_pin;
	p_motor->B_pin = B_pin;
	p_motor->p_timer_pwm = p_timer_pwm;
	p_motor->timer_channel = timer_channel;

}
