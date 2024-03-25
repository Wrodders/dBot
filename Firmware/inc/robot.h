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
    PID motionCtrl;
    float trgtAngVel;
}Robot; // Deferential Drive Mobile TWSB

static Robot robotInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    Robot bot = {
        .mpu6050 =  mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .imu =  imuInit(0.5f, 0.01f),
        .comp = compFiltInit(CTRL_PERIOD * MS_TO_S, 0.05f),
        .balanceCtrl = pidInit(-VEL_MAX, VEL_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S)),
        .motionCtrl = pidInit(-VBAT_MAX, VEL_MAX, VEL_KP, VEL_KI, VEL_KD, (CTRL_PERIOD * MS_TO_S))
    };

    return bot;
}

static inline void robotTrgtLinVel(Robot *bot, float linVel){bot->motionCtrl.target = linVel;}
static inline void robotTrgtAngVel(Robot *bot, float angVel){bot->trgtAngVel = angVel;}

#endif // ROBOT_H