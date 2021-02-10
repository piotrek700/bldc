#include <sdk/ahrs.h>
#include <sdk/utils.h>
#include <math.h>

CCMRAM_VARIABLE static float q1w = 1.0f, q1x = 0.0f, q1y = 0.0f, q1z = 0.0f;
CCMRAM_VARIABLE static float q2w = 1.0f, q2x = 0.0f, q2y = 0.0f, q2z = 0.0f;

CCMRAM_VARIABLE static float beta = AHRS_BETA_INIT;
CCMRAM_VARIABLE static float sampling_t = 1.0f / AHRS_SAMPLE_FREQUENCY_HZ;

void ahrs_set_beta(float value) {
	beta = value;
}

void ahrs_set_sampling_frequency(float value) {
	sampling_t = 1.0f / value;
}

CCMRAM_FUCNTION void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	//Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1x * gx - q1y * gy - q1z * gz);
	qDot2 = 0.5f * (q1w * gx + q1y * gz - q1z * gy);
	qDot3 = 0.5f * (q1w * gy - q1x * gz + q1z * gx);
	qDot4 = 0.5f * (q1w * gz + q1x * gy - q1y * gx);

	//Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		//Normalize accelerometer measurement
		recipNorm = fast_inv_sqrtf(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		//Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q1w;
		_2q1 = 2.0f * q1x;
		_2q2 = 2.0f * q1y;
		_2q3 = 2.0f * q1z;
		_4q0 = 4.0f * q1w;
		_4q1 = 4.0f * q1x;
		_4q2 = 4.0f * q1y;
		_8q1 = 8.0f * q1x;
		_8q2 = 8.0f * q1y;
		q0q0 = q1w * q1w;
		q1q1 = q1x * q1x;
		q2q2 = q1y * q1y;
		q3q3 = q1z * q1z;

		//Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q1y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q1z - _2q1 * ax + 4.0f * q2q2 * q1z - _2q2 * ay;
		recipNorm = fast_inv_sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		//Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	//Integrate rate of change of quaternion to yield quaternion
	q1w += qDot1 * sampling_t;
	q1x += qDot2 * sampling_t;
	q1y += qDot3 * sampling_t;
	q1z += qDot4 * sampling_t;

	//Normalize quaternion
	//TODO it is not necessary to normalize, normalization once again in ahrs_rotate_45
	recipNorm = fast_inv_sqrtf(q1w * q1w + q1x * q1x + q1y * q1y + q1z * q1z);
	q1w *= recipNorm; //w
	q1x *= recipNorm; //x
	q1y *= recipNorm; //y
	q1z *= recipNorm; //z
}

/*Rotation 45deg
 * https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
 * q_rot=[M_SQRT1_2, 0, 0, M_SQRT1_2], M_SQRT1_2=1/sqrt(2)
 * 
 * 
 * q_out=[
 * 			w=	q1w*q2w	-	q1x*q2x	-	q1y*q2y	-	q1z*q2z
 * 			x=	q1w*q2x	+	q1x*q2w	-	q1y*q2z +	q1z*q2y
 * 			y=	q1w*q2y	+	q1x*q2z	+	q1y*q2w	-	q1z*q2x
 * 			z=	q1w*q2z	-	q1x*q2y	+	q1y*q2x	+	q1z*q2w
 * ]
 */

CCMRAM_FUCNTION void ahrs_rotate_45(void) {
	q2w = AHRS_W_COS_45_2 * q1w - AHRS_Z_SIN_45_2 * q1z;
	q2x = AHRS_W_COS_45_2 * q1x + AHRS_Z_SIN_45_2 * q1y;
	q2y = AHRS_W_COS_45_2 * q1y - AHRS_Z_SIN_45_2 * q1x;
	q2z = AHRS_W_COS_45_2 * q1z + AHRS_Z_SIN_45_2 * q1w;

	float recipNorm = fast_inv_sqrtf(q2w * q2w + q2x * q2x + q2y * q2y + q2z * q2z);
	q2w *= recipNorm; //w
	q2x *= recipNorm; //x
	q2y *= recipNorm; //y
	q2z *= recipNorm; //z
}

void ahrs_rotate_0(void) {
	q2w = q1w;
	q2x = q1x;
	q2y = q1y;
	q2z = q1z;
}

//Tait-bryan http://www.chrobotics.com/library/understanding-quaternions
CCMRAM_FUCNTION float ahrs_get_yaw(void) {
	return fast_atan2f(2.0f * (q2y * q2z + q2w * q2x), q2w * q2w - q2x * q2x - q2y * q2y + q2z * q2z) * 180.0f * (float) M_1_PI;
}

CCMRAM_FUCNTION float ahrs_get_pitch(void) {
	return asinf(2.0f * (q2x * q2z - q2w * q2y)) * 180.0f * (float) M_1_PI;
}

CCMRAM_FUCNTION float ahrs_get_roll(void) {
	return -fast_atan2f(2.0f * (q2x * q2y + q2w * q2z), q2w * q2w + q2x * q2x - q2y * q2y - q2z * q2z) * 180.0f * (float) M_1_PI;
}

float *ahrs_get_q1(void) {
	static float q1[4] = { 0, 0, 0, 0 };

	q1[0] = q1w;
	q1[1] = q1x;
	q1[2] = q1y;
	q1[3] = q1z;

	return q1;
}

float *ahrs_get_q2(void) {
	static float q2[4] = { 0, 0, 0, 0 };

	q2[0] = q2w;
	q2[1] = q2x;
	q2[2] = q2y;
	q2[3] = q2z;

	return q2;
}
