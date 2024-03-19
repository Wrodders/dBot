#ifndef ROBOT_H
#define ROBOT_H

#include "../common/common.h"
#include "motor.h"
#include "pid.h"


/*
Kinematics Controller for TWSB-Robot Model of robot. 

Robot - Differential Driver Mobile Robot 
    Models and Controls the kinematics of the Robot in the 2D Plane 
    Each motor has a PD Speed controller, utilizes Quadrupole Encoder Timer
    
    The model maps Target Linear & Angular Velocity to Motor Velocities
    by the transfer function :

    q = [ x_dot y_dot theta_dot ]^T * [wR wR]^T

    wR = wL = V * Ki/(s^2(LaRa) +s(RaJm + BmLa) + KtKe + RaBm)        
*/  


typedef struct Robot{
    Motor motorL, motorR;
    Encoder encL, encR;


    const float wheelR; // R
    const float wheelBase; // 2L
    
    // Pose
    float angularV;
    float linearV;
    float posX, posY;
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
    };

    motorConfig(&bot.motorL, &bot.encL, VBAT_MAX, 0.0f, true, BETA_SPEED);
    motorConfig(&bot.motorR, &bot.encR, VBAT_MAX, 0.0f, false,  BETA_SPEED);

    driverEnable(&bot.motorL.drv); // enable DRV8833 & pwm
    driverEnable(&bot.motorR.drv); // enable DRV8833 & pwm

    motorStop(&bot.motorL);
    motorStop(&bot.motorR);
    

    return bot;
}


static void robotSpeedCtrl(Robot *bot){
    //@Brief: Main Speed Control Process 
    //@Description: Drives the mobile robot according to 

    // shortens calculations
    Motor* const mL = &bot->motorL;
    Motor* const mR = &bot->motorR;
    // Get target Wheel Speeds
    // Use old speed if no new speed set
    motorSpeedCtrl(mL);
    motorSpeedCtrl(mR);
    
}

static void robotDiffDrive(Robot* bot, const float linVel, const float angVel){
    //@Brief: Drive Mobile robot in Differential Drive Configuration
    //@Description: Computes Inverse Kinematics of Robot Model, Applies Speed Ramp via RingBuffer

    float wLTarget = linVel + (angVel * bot->wheelBase); // left wheel angular speed rad/s
    float wRTarget = linVel - (angVel * bot->wheelBase); // right wheel angular speed rad/s
}


static void robotTankDrive(Robot* bot, const float velL, const float velR){
    //@Brief: Drive Mobile Robot in Tank Drive Configuration
    //@Description: Sets Motor Voltage independently.
    //@Param: velL is a % 0-1

    float vL = _clamp(velL, -1, 1) * VBAT_MAX; // convert % to voltage
    float vR = _clamp(velR, -1, 1) * VBAT_MAX;
    
    motorSetSpeed(&bot->motorL, vL);
    motorSetSpeed(&bot->motorR, vR);
 //
    
}

#endif // ROBOT_H