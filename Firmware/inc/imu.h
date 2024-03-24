#ifndef IMU_H
#define IMU_H

#include "../drivers/MPU6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

typedef struct IMU{
    // Estimated Euler Angles
    float roll;
    float pitch;
    float yaw;

    // Low Pass IIR Filter 
    vector_t filtAccel; 
    vector_t filtGyro;
    float aAccel; 
    float aGyro;
}IMU; 


static IMU imuInit(float aAccel, float aGyro){
    IMU imu = {0};
    imu.aAccel = aAccel;
    imu.aGyro = aGyro;
    return imu;
}

static void imuLPF(IMU *imu, vector_t *accel, vector_t *gyro){
    // first order IIR filter

    imu->filtAccel.x = (imu->aAccel * imu->filtAccel.x) + (1.0f - imu->aAccel) * accel->x;
    imu->filtAccel.y = (imu->aAccel * imu->filtAccel.y) + (1.0f - imu->aAccel) * accel->y;
    imu->filtAccel.z = (imu->aAccel * imu->filtAccel.z) + (1.0f - imu->aAccel) * accel->z;

    imu->filtGyro.x = (imu->aGyro * imu->filtGyro.x) + (1.0f - imu->aGyro) * gyro->x;
    imu->filtGyro.y = (imu->aGyro * imu->filtGyro.y) + (1.0f - imu->aGyro) * gyro->y;
    imu->filtGyro.z = (imu->aGyro * imu->filtGyro.z) + (1.0f - imu->aGyro) * gyro->z;
}


static void imuRawEuler(IMU *imu){

    // roll (x-axis rotation)
    float ax2 = imu->filtAccel.x * imu->filtAccel.x;
    float ay2 = imu->filtAccel.y * imu->filtAccel.y;
    float az2 = imu->filtAccel.z * imu->filtAccel.z;


    imu->pitch = atanf(imu->filtAccel.y * invSqrt(ax2 + az2));
    imu->roll = atan(imu->filtAccel.x * invSqrt(ay2 + az2));
}


// ********* Complementary Filter ******** // ************************************************// 
typedef struct CompFilt{
    float dt; // s
    float a;
}CompFilt;

static CompFilt compFiltInit(float dt, float alpha){
    //@Brief Initializes Complementary filter
    CompFilt comp = {
        .a = alpha,
        .dt = dt
    };
    return comp;
}
static void compFilter(CompFilt * filt, IMU *imu){
    //@Brief: Complementary Filter Converts imu data into Euler Angles
    vector_t *accel = &imu->filtAccel;
    vector_t *gyro = &imu->filtGyro;

    double pitchAcc = atan2(accel->y, sqrt((accel->x * accel->x) + (accel->z * accel->z))); //  angle between horizontal plane and accel vector
    double rollAcc = atan2(-accel->x, accel->z);
    // apply integrator
    imu->pitch += gyro->x * filt->dt;
    imu->roll += gyro->y * filt->dt;
    imu->yaw += gyro->z * filt->dt;
    // apply bias
    imu->pitch = filt->a * imu->pitch + (1 -filt->a) * pitchAcc;
    imu->roll = filt->a * imu->roll + (1 -filt->a) * rollAcc;
}



#endif // IMU_H