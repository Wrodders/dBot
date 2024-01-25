#ifndef IMU_H
#define IMU_H


#include "../sensors/MPU6050.h"
#include "madgwick.h"


typedef struct IMU{
    float roll;
    float pitch;
    float yaw;
}IMU; 

#define DT 0.001 // samplegn inteval 50ms
#define ALPHA 0.58 //


static void updateOrientation(IMU *imu, vector_t *accel, vector_t *gyro){
    double pitchAcc = atan2(accel->y, sqrt(accel->x * accel->x + accel->z * accel->z));
    double rollAcc = atan2(-accel->x, accel->z);

    imu->pitch += gyro->x * DT;
    imu->roll += gyro->y * DT;
    imu->yaw += gyro->z * DT;

    imu->pitch = ALPHA * imu->pitch + (1 -ALPHA) * pitchAcc;
    imu->roll = ALPHA * imu->roll + (1 -ALPHA) * rollAcc;
}




#endif // IMU_H