#include <sdk/pid.h>

void pid_set_param(Pid_t *p_pid, float kp, float ki, float kd, float out_limit, float d_filter_coeff) {
	p_pid->kp = kp;
	p_pid->ki = ki;
	p_pid->kd = kd;
	p_pid->out_limit = out_limit;

	if (d_filter_coeff > 1.0f) {
		d_filter_coeff = 1.0f;
	}

	if (d_filter_coeff < 0.0f) {
		d_filter_coeff = 0.0f;
	}

	p_pid->d_filter_coeff = d_filter_coeff;

	p_pid->integral = 0.0f;
	//TODO uncomment
	//pid_reset(pid);
}

void pid_reset(Pid_t *p_pid) {
	p_pid->d_filtered = 0.0f;
	p_pid->deriv = 0.0f;
	p_pid->integral = 0.0f;
}

float pid_control_pid(Pid_t *p_pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * p_pid->kp;

	//D
	float d = (e - p_pid->deriv) * p_pid->kd / dt;
	p_pid->deriv = e;

	//Filter D
	p_pid->d_filtered = p_pid->d_filtered * p_pid->d_filter_coeff + (1.0f - p_pid->d_filter_coeff) * d;

	//PD
	float pd = p + p_pid->d_filtered;

	//PID
	float out = pd + p_pid->integral + e * p_pid->ki * dt;

	//PID wind-up
	if (out > p_pid->out_limit) {
		out = p_pid->out_limit;
	} else if (out < -p_pid->out_limit) {
		out = -p_pid->out_limit;
	}else{
		//I
		p_pid->integral += e * p_pid->ki * dt;
	}

	/*
	//I wind-up limit calculation MIN and MAX
	float i_max = MAX(pid->out_limit - pd, 0.0f);
	float i_min = MIN(-pid->out_limit - pd, 0.0f);


	//I wind-up
	if (pid->integral > i_max) {
		pid->integral = i_max;
	} else if (pid->integral < i_min) {
		pid->integral = i_min;
	}

	//PID
	float out = pd + pid->integral;

	//PID wind-up
	if (out > pid->out_limit) {
		out = pid->out_limit;
	} else if (out < -pid->out_limit) {
		out = -pid->out_limit;
	}
*/
	return out;
}

float pid_control_pi(Pid_t *p_pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * p_pid->kp;

	//I wind-up limit calculation MIN and MAX
	float i_max = MAX(p_pid->out_limit - p, 0.0f);
	float i_min = MIN(-p_pid->out_limit - p, 0.0f);

	//I
	p_pid->integral += e * p_pid->ki * dt;

	//I wind-up
	if (p_pid->integral > i_max) {
		p_pid->integral = i_max;
	} else if (p_pid->integral < i_min) {
		p_pid->integral = i_min;
	}

	//PID
	float out = p + p_pid->integral;

	//PID wind-up
	if (out > p_pid->out_limit) {
		out = p_pid->out_limit;
	} else if (out < -p_pid->out_limit) {
		out = -p_pid->out_limit;
	}

	return out;
}

float pid_control_pd(Pid_t *p_pid, float value, float ref, float dt) {
	//Error, p
	float e = ref - value;
	float p = e * p_pid->kp;

	//D
	float d = (e - p_pid->deriv) * p_pid->kd / dt;
	p_pid->deriv = e;

	//Filter D
	p_pid->d_filtered = p_pid->d_filtered * p_pid->d_filter_coeff + (1.0f - p_pid->d_filter_coeff) * d;

	//PID
	float out = p + p_pid->d_filtered;

	//PID wind-up
	if (out > p_pid->out_limit) {
		out = p_pid->out_limit;
	} else if (out < -p_pid->out_limit) {
		out = -p_pid->out_limit;
	}

	return out;
}
