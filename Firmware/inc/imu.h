#ifndef IMU_H
#define IMU_H

#include "../drivers/MPU6050.h"

typedef struct IMU{
    MPU6050 sensor;
    // Low Pass IIR Filter 
    struct{
        float aAccel;
        float aGyro;
        vector_t accel; 
        vector_t gyro;
    }lpf;
    struct{// Complementary Filter
        const float dt;
        const float a;
    }comp;
    // Estimated Euler Angles
    float roll;
    float pitch;
    float yaw;
}IMU; 

static IMU imuInit(float aAccel, float aGyro, float compAlpha, float dt){
    IMU imu = {
        .sensor = mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .lpf = {.aAccel = aAccel, .aGyro = aGyro},
        .comp = {.a = compAlpha, .dt = dt},      
    };
    return imu;
}

static void imuLPF(IMU *imu, vector_t *accel, vector_t *gyro){
    // first order IIR filter
    imu->lpf.accel.x = (imu->lpf.aAccel * imu->lpf.accel.x) + (1.0f - imu->lpf.aAccel) * accel->x;
    imu->lpf.accel.y = (imu->lpf.aAccel * imu->lpf.accel.y) + (1.0f - imu->lpf.aAccel) * accel->y;
    imu->lpf.accel.z = (imu->lpf.aAccel * imu->lpf.accel.z) + (1.0f - imu->lpf.aAccel) * accel->z;

    imu->lpf.gyro.x = (imu->lpf.aGyro * imu->lpf.gyro.x) + (1.0f - imu->lpf.aGyro) * gyro->x;
    imu->lpf.gyro.y = (imu->lpf.aGyro * imu->lpf.gyro.y) + (1.0f - imu->lpf.aGyro) * gyro->y;
    imu->lpf.gyro.z = (imu->lpf.aGyro * imu->lpf.gyro.z) + (1.0f - imu->lpf.aGyro) * gyro->z;
}

static void imuRawEuler(IMU *imu){
    //@Brief: Computes Euler from raw accelerometer data

    // roll (x-axis rotation)
    float ax2 = imu->lpf.accel.x * imu->lpf.accel.x;
    float ay2 = imu->lpf.accel.y * imu->lpf.accel.y;
    float az2 = imu->lpf.accel.z * imu->lpf.accel.z;
    imu->pitch = atanf(imu->lpf.accel.y * invSqrt(ax2 + az2));
    imu->roll = atan(imu->lpf.accel.x * invSqrt(ay2 + az2));
}

static void imuCompFilt(IMU *imu){
    //@Brief: Complementary Filter Converts imu data into Euler Angles
    vector_t *accel = &imu->lpf.accel;
    vector_t *gyro = &imu->lpf.accel;

    double pitchAcc = atan2(accel->y, sqrt((accel->x * accel->x) + (accel->z * accel->z))); //  angle between horizontal plane and accel vector
    double rollAcc = atan2(-accel->x, accel->z);
    // apply integrator
    imu->pitch += gyro->x * imu->comp.dt;
    imu->roll += gyro->y * imu->comp.dt;
    imu->yaw += gyro->z * imu->comp.dt;
    // apply bias
    imu->pitch = imu->comp.a * imu->pitch + (1 -imu->comp.a) * pitchAcc;
    imu->roll = imu->comp.a * imu->roll + (1 -imu->comp.a) * rollAcc;
}

static void imuRunFusion(IMU *imu){
    //@Brief: Run Sensor Fusion Algorithms 6-Axis IMU
    mpu6050Read(&imu->sensor);
    imuLPF(imu, &imu->sensor.accel, &imu->sensor.gyro);
    imuCompFilt(imu); // sensor fuction euler angles
}

#endif // IMU_H