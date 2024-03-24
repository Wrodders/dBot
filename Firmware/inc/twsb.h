#ifndef ROBOT_H
#define ROBOT_H

#include "../common/common.h"

#include "imu.h"
#include "pid.h"

// ******* Two Wheel Self-Balancing  Mobile TWSB ********* // 
//@Brief: Controls TWSB Balance



typedef struct TWSB{
    MPU6050 mpu6050;
    IMU imu;
    CompFilt comp;

    PID balancer;

}TWSB; // Deferential Drive Mobile TWSB

static TWSB twsbInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    TWSB bBot = {
        .mpu6050 =  mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .imu =  imuInit(0.5f, 0.01f),
        .comp = compFiltInit(BALANCE_PERIOD * MS_TO_S, 0.05f),
        .balancer = pidInit(-VEL_MAX, VEL_MAX, BAL_KP, BAL_KI, BAL_KD, (BALANCE_PERIOD * MS_TO_S))

    };
    
    return bBot;
}

static void twsbSetRef(TWSB *bBot, float angle){
    //@Brief: Sets Balancing PID Reference
    bBot->balancer.target = angle;
}

static void twsbCalcTheta(TWSB *bBot){
    //@Brief: Calculate Robots Pitch Angle

    mpu6050Read(&bBot->mpu6050);
    // apply digital LPF to raw measurements
    imuLPF(&bBot->imu, &bBot->mpu6050.accel, &bBot->mpu6050.gyro);
    compFilter(&bBot->comp, &bBot->imu);
}

static void twsbBalancer(TWSB *bBot){
    //@Brief: Run Balance PID Control Loop 
    //@Description: Outputs Target Mobile TWSB Linear Velocity

    // Calculate angle error 
    twsbCalcTheta(bBot);
    float mTheta = bBot->imu.roll; // updated at BALANCE_PERDIDO rate
    pidRun(&bBot->balancer, mTheta); // apply pid
    
    //robotDiffDrive(bot, bot->balancer.out, 0); // Inverse Kinematics
}

#endif // ROBOT_H