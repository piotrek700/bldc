#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

#define PID_STATIC_INIT(_pid_name, _kp, _ki, _kd, _out_limit, _d_filter_coeff)	\
static Pid _pid_name = {														\
	.kp = _kp,																	\
	.ki = _ki,																	\
	.kd = _kd,																	\
	.integral = 0,																\
	.deriv = 0,																	\
	.d_filtered = 0,															\
	.d_filter_coeff = _d_filter_coeff,											\
	.out_limit = _out_limit														\
}

typedef struct {
	float kp;
	float ki;
	float kd;

	float integral;
	float deriv;
	float d_filtered;

	//0..1 -> 0 not filtered...filtered 1
	float d_filter_coeff;
	float out_limit;
} Pid;

void pid_init(Pid *pid, float kp, float ki, float kd, float out_limit, float d_filter_coeff);

void pid_set_param(Pid *pid, float kp, float ki, float kd, float out_limit, float d_filter_coeff);

void pid_reset(Pid *pid);

float pid_control_pid(Pid *pid, float value, float ref, float dt);

float pid_control_pi(Pid *pid, float value, float ref, float dt);

float pid_control_pd(Pid *pid, float value, float ref, float dt);


#endif
