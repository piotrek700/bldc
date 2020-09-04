#include "pid.h"

void pid_init(Pid *pid, float kp, float ki, float kd, float out_limit, float d_filter_coeff) {
	pid_set_param(pid, kp, ki, kd, out_limit, d_filter_coeff);
	pid_reset(pid);
}

void pid_set_param(Pid *pid, float kp, float ki, float kd, float out_limit, float d_filter_coeff) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->out_limit = out_limit;
	pid->d_filter_coeff = d_filter_coeff;
}

void pid_reset(Pid *pid) {
	pid->d_filtered = 0.0f;
	pid->deriv = 0.0f;
	pid->integral = 0.0f;
}

float pid_control_pid(Pid *pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * pid->kp;

	//D
	float d = (e - pid->deriv) * pid->kd / dt;
	pid->deriv = e;

	//Filter D
	pid->d_filtered = pid->d_filtered * pid->d_filter_coeff + (1.0f - pid->d_filter_coeff) * d;

	//PD
	float pd = p + pid->d_filtered;

	//I wind-up limit calculation MIN and MAX
	float i_max = MAX(pid->out_limit - pd, 0.0f);
	float i_min = MIN(-pid->out_limit - pd, 0.0f);

	//I
	pid->integral += e * pid->ki * dt;

	//I wind-up
	if (pid->integral > i_max) {
		pid->integral = i_max;
	} else if (pid->integral < i_min) {
		pid->integral = i_min;
	}

	//PID
	float out = p + pid->integral + d;

	//PID wind-up
	if (out > pid->out_limit) {
		out = pid->out_limit;
	} else if (out < -pid->out_limit) {
		out = -pid->out_limit;
	}

	return out;
}

float pid_control_pi(Pid *pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * pid->kp;

	//I wind-up limit calculation MIN and MAX
	float i_max = MAX(pid->out_limit - p, 0.0f);
	float i_min = MIN(-pid->out_limit - p, 0.0f);

	//I
	pid->integral += e * pid->ki * dt;

	//I wind-up
	if (pid->integral > i_max) {
		pid->integral = i_max;
	} else if (pid->integral < i_min) {
		pid->integral = i_min;
	}

	//PID
	float out = p + pid->integral;

	//PID wind-up
	if (out > pid->out_limit) {
		out = pid->out_limit;
	} else if (out < -pid->out_limit) {
		out = -pid->out_limit;
	}

	return out;
}

float pid_control_pd(Pid *pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * pid->kp;

	//D
	float d = (e - pid->deriv) * pid->kd / dt;
	pid->deriv = e;

	//Filter D
	pid->d_filtered = pid->d_filtered * pid->d_filter_coeff + (1.0f - pid->d_filter_coeff) * d;

	//PID
	float out = p  + d;

	//PID wind-up
	if (out > pid->out_limit) {
		out = pid->out_limit;
	} else if (out < -pid->out_limit) {
		out = -pid->out_limit;
	}

	return out;
}
