#ifndef _HAVE_PID_H_
#define _HAVE_PID_H_

#include <stdint.h>
#include <math.h>


typedef struct {
	float e_sum;
	float e_prev;
	float kP;
	float kI;
	float kD;
	float setpoint;
	float hysteresis;
}PID_t;


void PID_init(PID_t *p_PID, float kP, float kI, float kD);
float PID_feed(PID_t *p_PID, float x);
void PID_setpoint(PID_t *p_PID, float setpoint, float hysteresis);








#endif // _HAVE_PID_H_
