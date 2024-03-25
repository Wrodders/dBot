#ifndef ROBOT_H
#define ROBOT_H

#include "../common/common.h"

#include "imu.h"
#include "pid.h"
#include "ddmr.h"

// ******* Two Wheel Self-Balancing  Mobile TWSB ********* // 
//@Brief: Controls TWSB Balance



typedef struct Robot{
    MPU6050 mpu6050;
    IMU imu;
    CompFilt comp;
    PID balanceCtrl;
}Robot; // Deferential Drive Mobile TWSB

static Robot robotInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    Robot bot = {
        .mpu6050 =  mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .imu =  imuInit(0.5f, 0.01f),
        .comp = compFiltInit(CTRL_PERIOD * MS_TO_S, 0.05f),
        .balanceCtrl = pidInit(-VEL_MAX, VEL_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S))
    };

    bot.balanceCtrl.target = BAL_THETA;
    
    return bot;
}

static inline void robotSetTheta(Robot *bot, float angle){bot->balanceCtrl.target = angle;}


static void robotCalTheta(Robot *bot){
    //@Brief: Calculate Robots Pitch Angle

    mpu6050Read(&bot->mpu6050);
    // apply digital LPF to raw measurements
    imuLPF(&bot->imu, &bot->mpu6050.accel, &bot->mpu6050.gyro);
    compFilter(&bot->comp, &bot->imu); // sensor fuction euler angles
}

#endif // ROBOT_H