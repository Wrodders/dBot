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


    PID balancer;
    State target;
}Robot; // Deferential Drive Mobile TWSB

static Robot robotInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    Robot bot = {
        .mpu6050 =  mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .imu =  imuInit(0.5f, 0.01f),
        .comp = compFiltInit(CTRL_PERIOD * MS_TO_S, 0.05f),
        .balancer = pidInit(-VEL_MAX, VEL_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S))

    };
    
    return bot;
}

static void robotSetTheta(Robot *bot, float angle){
    //@Brief: Sets Balancing PID Reference
    bot->balancer.target = angle;
}

static void robotNewState(Robot *bot, float angVel, float linVel, float posX, float posY, float theta){
    bot->target.angularV = angVel;
    bot->target.linearV = linVel;
    bot->target.posX = posX;
    bot->target.posY = posY;
    bot->balancer.target = theta;

}

static void robotNewLinVel(Robot *bot, float vel){bot->target.linearV = vel;}
static void robotNewAngVel(Robot *bot, float vel){bot->target.angularV = vel;}

static void robotStop(Robot *bot){
    // Stops robot,
    // Zeros robots body velocity targets
    robotNewLinVel(bot, 0);
    robotNewAngVel(bot, 0);
}

static void robotCalTheta(Robot *bot){
    //@Brief: Calculate Robots Pitch Angle

    mpu6050Read(&bot->mpu6050);
    // apply digital LPF to raw measurements
    imuLPF(&bot->imu, &bot->mpu6050.accel, &bot->mpu6050.gyro);
    compFilter(&bot->comp, &bot->imu);
}

static void robotBalancer(Robot *bot){
    //@Brief: Run Balance PID Control Loop 
    //@Description: Outputs Target Mobile TWSB Linear Velocity

    // Calculate angle error 
    robotCalTheta(bot);
    float mTheta = bot->imu.roll; // updated at BALANCE_PERDIDO rate
    if(_fabs(mTheta) > 0.7f){robotStop(bot);}
    else{
        pidRun(&bot->balancer, mTheta); // apply pid
        robotNewLinVel(bot, bot->balancer.out); // update target
    }
}

#endif // ROBOT_H