#ifndef MADJWICK_H
#define MADJWICK_H

#include <math.h>
#include "../common/common.h"
// ********* Madgwick Filter Functions *****************************************************// 

#define MADGWICK_FREQ 500
#define MADGWICK_BETA 0.1f

typedef struct MadgwickFilter{
    volatile float beta;
    volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    float sampleFreq;
}MadgwickFilter;

static float invSqrt(float x){
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    float halfx = 0.5f * x;
    float y = 0.0f;
    y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void computeAngles(vector_t *gyro, vector_t *accel, float *roll, float *pitch){

    // roll (x-axis rotation)
    float ax2 = accel->x * accel->x;
    float ay2 = accel->y * accel->y;
    float az2 = accel->z * accel->z;


    *roll = atan(accel->y * invSqrt(ax2 + az2));
    // pitch (y-axis rotation)
    *pitch = atan(accel->x * invSqrt(ay2 + az2));
    return;
}
bool imu_madgwick_get_angles(MadgwickFilter *filter, float *roll, float *pitch, float *yaw) {
  if (!filter) {
    return false;
  }
  if (roll) {
    *roll = asinf(-2.0f * (filter->q1 * filter->q3 - filter->q0 * filter->q2));
  }
  if (pitch) {
    *pitch = atan2f(filter->q0 * filter->q1 + filter->q2 * filter->q3, 0.5f - filter->q1 * filter->q1 - filter->q2 * filter->q2);
  }
  if (yaw) {
    *yaw = atan2f(filter->q1 * filter->q2 + filter->q0 * filter->q3, 0.5f - filter->q2 * filter->q2 - filter->q3 * filter->q3);
  }
  return true;
}

static void MadgwickAHRSupdateIMU(MadgwickFilter *mf, float gx, float gy, float gz, float ax, float ay, float az) {
    //Madjwick filter 
    volatile float q0 = mf->q0, q1 = mf->q1, q2 = mf->q2, q3 = mf->q3;   
    volatile float beta = mf->beta;
    volatile float sampleFreq = mf->sampleFreq;

    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    // Update filter
    mf->q0 = q0;
    mf->q1 = q1;
    mf->q2 = q2;
    mf->q3 = q3;
    return;
}


static MadgwickFilter initMadgwick(float freq, float beta){
    MadgwickFilter mf;

    mf.beta = beta;
    mf.sampleFreq = freq;
    mf.q0 = 1.0f;
    mf.q1 = 0.0f;
    mf.q2 = 0.0f;
    mf.q3 = 0.0f;
    return mf;
}


#endif // MADGWICK_H