#ifndef MADGWICK_H
#define MADGWICK_H

#include <math.h>

// ********* Madgwick Filter ******** // ************************************************// 
typedef struct MadgwickFilter {
  float    beta;
  float    q0;
  float    q1;
  float    q2;
  float    q3;
  float    freq;
  float    period;
  uint32_t counter;
}MadgwickFilter;

static bool madgwickConfig(MadgwickFilter *filter, float freq, float beta){
    if (!filter) {
    return false;
  }
  filter->beta     = beta;
  filter->freq     = freq; // hz
  filter->period = 1.0f / freq;
  return true;
}

static bool madgwickReset(MadgwickFilter *filter){
     if (!filter) {
    return false;
  }
  filter->q0 = 1.0f;
  filter->q1 = 0.0f;
  filter->q2 = 0.0f;
  filter->q3 = 0.0f;
  filter->counter = 0;
  return true;
}


static MadgwickFilter madgwickInit(void) {
    MadgwickFilter filter;
    madgwickConfig(&filter, 500.0f, 0.1f);
    madgwickReset(&filter);
    return filter;
}


static bool imuMadgwickFilt(MadgwickFilter *filter, IMU *imu){

    float gx = imu->filtGyro.x;
    float gy = imu->filtGyro.y;
    float gz = imu->filtGyro.z;

    float ax = imu->filtAccel.x;
    float ay = imu->filtAccel.y;
    float az = imu->filtAccel.z;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // No need to check filter pointer -- it's checked by _update()

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
    qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalize accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax       *= recipNorm;
    ay       *= recipNorm;
    az       *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * filter->q0;
    _2q1 = 2.0f * filter->q1;
    _2q2 = 2.0f * filter->q2;
    _2q3 = 2.0f * filter->q3;
    _4q0 = 4.0f * filter->q0;
    _4q1 = 4.0f * filter->q1;
    _4q2 = 4.0f * filter->q2;
    _8q1 = 8.0f * filter->q1;
    _8q2 = 8.0f * filter->q2;
    q0q0 = filter->q0 * filter->q0;
    q1q1 = filter->q1 * filter->q1;
    q2q2 = filter->q2 * filter->q2;
    q3q3 = filter->q3 * filter->q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * filter->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * filter->q3 - _2q1 * ax + 4.0f * q2q2 * filter->q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);             // normalize step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= filter->beta * s0;
    qDot2 -= filter->beta * s1;
    qDot3 -= filter->beta * s2;
    qDot4 -= filter->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    filter->q0 += qDot1 * filter->period;
    filter->q1 += qDot2 * filter->period;
    filter->q2 += qDot3 * filter->period;
    filter->q3 += qDot4 * filter->period;

    // Normalize quaternion
    recipNorm   = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;

    filter->counter++;
    return true;

}

static bool madgwickGetEuler(IMU *imu, MadgwickFilter *filter){
    if (!filter) {return false;}
    // Calc Angles
    imu->roll = asinf(-2.0f * (filter->q1 * filter->q3 - filter->q0 * filter->q2));
    imu->pitch = atan2f(filter->q0 * filter->q1 + filter->q2 * filter->q3, 0.5f - filter->q1 * filter->q1 - filter->q2 * filter->q2);
    imu->yaw = atan2f(filter->q1 * filter->q2 + filter->q0 * filter->q3, 0.5f - filter->q2 * filter->q2 - filter->q3 * filter->q3);
    
    return true;
}


#endif // MADGWICK_H