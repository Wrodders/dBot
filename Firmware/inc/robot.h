#ifndef ROBOT_H
#define ROBOT_H

#include "../common/common.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"


/*
Kinematics Controller for TWSB-Robot Model of robot. 

Robot - Differential Driver Mobile Robot 
    Models and Controls the kinematics of the TWSB Robot in the 2D Plane 
    Provides a DDMR Controllable Robot via cascaded PID 

    Forward Kinematics:
    Va = R * (wR + wL) / 2
    Wa = 2R * (wR - wL) / L

    Inverse Kinematics: 
    wR = Va - (Wa * L 
    wL = Va + Wa * L)
    
*/  


typedef struct Robot{
    Motor motorL, motorR;
    Encoder encL, encR;

    MPU6050 mpu6050;
    IMU imu;
    CompFilt comp;

    PID balancer;


    const float wheelR; // R
    const float wheelBase; // L
    
    // kinematic state
    struct{
    float angularV;
    float linearV;
    float posX, posY;
    }state;
}Robot; // Deferential Drive Mobile Robot


static Robot robotInit(void){
    //@Brief: Initializes Peripherals Sensors and Controllers for Low Level Robot Robot
    //@Description: Sets Up Low Level Differential Drive Robot Model with constant speed motors  
    //Constant speed required by kinematics model implemented with PI Controller though Fixed Time Task
    
    Robot bot = {
        .wheelBase = WHEEL_BASE, 
        .wheelR = WHEEL_RADIUS,        

        .encL = encoderInit(ENC_L_TIM, UINT16_MAX, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT, ENC_L_AF),
        .motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT),
            
        .encR = encoderInit(ENC_R_TIM, UINT16_MAX, ENC_R_A, ENC_R_PORT, ENC_R_B, ENC_R_PORT, ENC_R_AF),
        .motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT),
        
        .mpu6050 =  mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA),
        .imu =  imuInit(0.5f, 0.01f),
        .comp = compFiltInit(BALANCE_PERIOD * MS_TO_S, 0.05f)

        };



    motorConfig(&bot.motorL, &bot.encL, VBAT_MAX, 0.0f, true, BETA_SPEED);
    motorConfig(&bot.motorR, &bot.encR, VBAT_MAX, 0.0f, false,  BETA_SPEED);

    motorDrvEn(&bot.motorL); // enable DRV8833 & pwm
    motorDrvEn(&bot.motorR); // enable DRV8833 & pwm

    motorStop(&bot.motorL);
    motorStop(&bot.motorR);
    

    return bot;
}


static void robotOdometry(Robot *bot){
    //@Brief: Calculate Mobile Robots Kinematic State
    //@Description: xdot = linVel * cos(theta)
    //              ydot = linVel * sin(theta)
    //              thetadot = angVel
    // Integrating these values we can obtain the Mobile Robots Pose q = [x y theta]^T
    // Assuming the constant acceleration.

    //  [linV angV]^T =  R/2 * [1 1; 1/L -1/L]^T * [wR wL]^T
    //  x(t) = (vR + vL)/(vR -vL) * w/2 * sin(t*(vR - VL)/w)
    //  y(t) = (vR + vL)/(vR -vL) * w/2 * cos(t*(vR - VL)/w) + (vR + vL)/(vR - vL) * w/2

    bot->state.linearV = (bot->wheelR / 2) * (bot->motorL.angularVel + bot->motorR.angularVel);
    bot->state.angularV = (bot->wheelR / (2* bot->wheelBase)) * (bot->motorL.angularVel - bot->motorR.angularVel);

  

}

static void robotDiffDrive(Robot* bot, const float linVel, const float angVel){
    //@Brief: Drive Mobile robot in Differential Drive Configuration
    //@Description: Computes Inverse Kinematics of Robot Model.
    //              

    float wLTarget = (linVel*MPS_TO_RPS) + (angVel * 2 * bot->wheelBase); // left wheel angular speed rad/s
    float wRTarget = (linVel*MPS_TO_RPS) - (angVel * 2 * bot->wheelBase); // right wheel angular speed rad/s
    motorSetSpeed(&bot->motorL,wLTarget);
    motorSetSpeed(&bot->motorR,wRTarget);
}


static void robotTankDrive(Robot* bot, const float pwrL, const float pwrR){
    //@Brief: Drive Mobile Robot in Tank Drive Configuration
    //@Description: Wheel Power Sets Motor Voltage Independently 

    float vL = _clamp(pwrL, -1, 1) * VBAT_MAX; // convert % to voltage
    float vR = _clamp(pwrR, -1, 1) * VBAT_MAX;
    motorSetSpeed(&bot->motorL, vL);
    motorSetSpeed(&bot->motorR, vR);
}

static void robotCalPitch(Robot *bot){
    //@Brief: Calculate Robots Pitch Angle

    mpu6050Read(&bot->mpu6050);
    // apply digital LPF to raw measurements
    imuLPF(&bot->imu, &bot->mpu6050.accel, &bot->mpu6050.gyro);
    compFilter(&bot->comp, &bot->imu);
}

static void robotBalance(Robot *bot){
    //@Brief: Run Balance PID Control Loop 
    //@Description: Outputs Target Mobile Robot Linear Velocity

    // Calculate angle error 
    robotCalPitch(bot);
    float mPitch = bot->imu.pitch; // updated at BALANCE_PERDIOD rate
    //pidRun(&bot->balancer, mPitch); // apply pid
    //robotDiffDrive(bot, bot->balancer.out, 0); // Inverse Kineatics
}










#endif // ROBOT_H