#ifndef AHRS_H_
#define AHRS_H_

#define AHRS_BETA_INIT				1.0f
#define AHRS_BETA_FINAL				0.1f
#define AHRS_SAMPLE_FREQUENCY_HZ 	500.0f

#define AHRS_W_COS_45_2				0.9238795325112867f
#define AHRS_Z_SIN_45_2				0.3826834323650897f

void ahrs_set_beta(float value);

void ahrs_set_sampling_frequency(float value);

void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az);

void ahrs_rotate_45(void);

void ahrs_rotate_0(void);

float ahrs_get_yaw(void);

float ahrs_get_pitch(void);

float ahrs_get_roll(void);

float *ahrs_get_q1(void);

float *ahrs_get_q2(void);

#endif
