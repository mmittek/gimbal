#include "PID.h"
#include "math.h"


void PID_init(PID_t *p_PID, float kP, float kI, float kD) {
	memset(p_PID, 0, sizeof(PID_t));
	p_PID->kP = kP;
	p_PID->kI = kI;
	p_PID->kD = kD;
	p_PID->e_prev = 0.0f;
	p_PID->e_sum = 0.0f;
	p_PID->setpoint = 0.0f;
}

void PID_setpoint(PID_t *p_PID, float setpoint) {
	p_PID->setpoint = setpoint;
}

float PID_feed(PID_t *p_PID, float x){
	float error = p_PID->setpoint - x;
	p_PID->e_sum += error;
	float out = p_PID->kP*error + p_PID->kI*(p_PID->e_sum) + p_PID->kD*(error-p_PID->e_prev);
	p_PID->e_prev = error;
	return out;
}
