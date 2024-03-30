#ifndef IMU_H
#define IMU_H

#include "../drivers/MPU6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105


typedef struct Kalman{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
}Kalman;

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
        const float a;
        float roll, pitch;
    }comp;
    
    Kalman kalX;
    Kalman kalY;
    float kalPitch, kalRoll;

    const float dt;
    // Estimated Euler Angles raw
    float roll;
    float pitch;
    float yaw;
}IMU; 




static IMU imuInit(float aAccel, float aGyro, float compAlpha, float dt){
    IMU imu = {
        .sensor = mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .lpf = {.aAccel = aAccel, .aGyro = aGyro},
        .comp = {.a = compAlpha},      
        .kalX = {.Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f},
        .kalY = {.Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f},
        .dt = dt

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
    imu->pitch = atan2f(-imu->lpf.accel.x, imu->lpf.accel.z) * RAD_TO_DEG;
    imu->roll = atan(imu->lpf.accel.y * invSqrt(ax2 + az2)) * RAD_TO_DEG;
}

static void imuCompFilt(IMU *imu){
    //@Brief: Complementary Filter Converts imu data into Euler Angles
    vector_t *accel = &imu->lpf.accel;
    vector_t *gyro = &imu->lpf.accel;

    double pitchAcc = atan2(accel->y, sqrt((accel->x * accel->x) + (accel->z * accel->z))); //  angle between horizontal plane and accel vector
    double rollAcc = atan2(-accel->x, accel->z);
    // apply integrator
    imu->comp.pitch += gyro->x * imu->dt;
    imu->comp.roll += gyro->y * imu->dt;
 
    // apply bias
    imu->comp.pitch = imu->comp.a * imu->comp.pitch + (1 -imu->comp.a) * imu->pitch;
    imu->comp.roll = imu->comp.a * imu->comp.roll + (1 -imu->comp.a) * imu->roll;
}

static float Kalman_getAngle(Kalman *kal, float newAngle, float newRate, float dt) {

    float rate = newRate - kal->bias;
    kal->angle += dt * rate;

    kal->P[0][0] += dt * (dt * kal->P[1][1] - kal->P[0][1] - kal->P[1][0] + kal->Q_angle);
    kal->P[0][1] -= dt * kal->P[1][1];
    kal->P[1][0] -= dt * kal->P[1][1];
    kal->P[1][1] += kal->Q_bias * dt;

    float S = kal->P[0][0] + kal->R_measure;
    float K[2];
    K[0] = kal->P[0][0] / S;
    K[1] = kal->P[1][0] / S;

    float y = newAngle - kal->angle;
    kal->angle += K[0] * y;
    kal->bias += K[1] * y;

    float P00_temp = kal->P[0][0];
    float P01_temp = kal->P[0][1];

    kal->P[0][0] -= K[0] * P00_temp;
    kal->P[0][1] -= K[0] * P01_temp;
    kal->P[1][0] -= K[1] * P00_temp;
    kal->P[1][1] -= K[1] * P01_temp;

    return kal->angle;
}

static void imuKalFilt(IMU *imu){

    float ax = imu->lpf.accel.x;
    float ay = imu->lpf.accel.y;
    float az = imu->lpf.accel.z;
    float ax2 = ax * ax;
    float az2 = az * az;


    // Kalman angle solv
    float roll;
    float rollsqrt = sqrt(ax2 + az2);

    if(rollsqrt != 0.0f){roll = atan(ay / rollsqrt) * RAD_TO_DEG;}
    else{roll = 0.0f;}
    float pitch = atan2(-ax, az ) * RAD_TO_DEG;
    if ((pitch < -90 && imu->kalPitch > 90) || (pitch > 90 && imu->kalPitch < -90)){
        imu->kalY.angle = pitch;
        imu->kalPitch = pitch;
    }
    else{
        imu->kalPitch = Kalman_getAngle(&imu->kalY, pitch, imu->lpf.gyro.y, imu->dt);
    }
    float xRate = imu->sensor.gyro.x;
    if (fabs(imu->kalPitch) > 90){
        imu->sensor.gyro.x *= -1;
    }   
    imu->kalRoll= Kalman_getAngle(&imu->kalX, roll, imu->lpf.gyro.y, imu->dt);

}

static void imuRunFusion(IMU *imu){
    //@Brief: Run Sensor Fusion Algorithms 6-Axis IMU
    mpu6050Read(&imu->sensor);
    imuLPF(imu, &imu->sensor.accel, &imu->sensor.gyro);
    imuRawEuler(imu); // Compute Raw angles
    imuCompFilt(imu); // Compute Angle With Complematery Filter

}

#endif // IMU_H