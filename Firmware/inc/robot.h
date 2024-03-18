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
    PID pidL;
    PID pidR;
    float beta; // LPF

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
    
    Robot ddmr = {
        .wheelBase = WHEEL_BASE, 
        .wheelR = WHEEL_RADIUS,
        .beta = 1,
        
        .encL = encoderHWInit(ENC_L_TIM, UINT16_MAX, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT, ENC_L_AF),
        .motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT),
        .pidL = pidInit(-VBAT_MAX, VBAT_MAX, KP,KI,KD, SPEEDCTRL_PERIOD * MS_TO_S),
    
        .encR = encoderHWInit(ENC_R_TIM, UINT16_MAX, ENC_R_A, ENC_R_PORT, ENC_R_B, ENC_R_PORT, ENC_R_AF),
        .motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT),
        .pidR = pidInit(-VBAT_MAX, VBAT_MAX, KP,KI,KD, SPEEDCTRL_PERIOD * MS_TO_S)
    };

    motorConfig(&ddmr.motorL, 12.0f, 9.0f, 0.0f, true, 0.5f);
    motorConfig(&ddmr.motorR, 12.0f, 9.0f, 0.0f, false, 0.5f);

    driverEnable(&ddmr.motorL.drv); // enable DRV8833 & pwm
    driverEnable(&ddmr.motorR.drv); // enable DRV8833 & pwm

    motorStop(&ddmr.motorL);
    motorStop(&ddmr.motorR);
    

    return ddmr;
}


static void robotSpeedCtrl(Robot *ddmr){
    //@Brief: Main Speed Control Process 
    //@Description: Drives the mobile robot according to 

    // shortens calculations
    Motor* const mL = &ddmr->motorL;
    Motor* const mR = &ddmr->motorR;
    Encoder* const encL = &ddmr->encL;
    Encoder* const encR = &ddmr->encR;
    PID* const pidL = &ddmr->pidL;
    PID* const pidR = &ddmr->pidR; 

    // Get target Wheel Speeds
    pidL->target = -3.0f; // placeholder
    pidR->target = -3.0f;

    // Compute Current wheel speed
    motorCalSpeed(mL, encL);
    motorCalSpeed(mR, encR);

    // Run PID Algorithm For Both Motors
    pidRun(&ddmr->pidL, mL->angularSpeed);
    pidRun(&ddmr->pidR, mR->angularSpeed);

    // Apply output to Motors
    motorSetVoltage(mL, ddmr->pidL.out);
    motorSetVoltage(mR, ddmr->pidR.out);
}

static void robotDiffDrive(Robot* ddmr, const float linVel, const float angVel){
    //@Brief: Drive Mobile robot in Differential Drive Configuration
    //@Description: Computes Inverse Kinematics of Robot Model, Applies Speed Ramp via RingBuffer

    float wLTarget = linVel + (angVel * ddmr->wheelBase); // left wheel angular speed rad/s
    float wRTarget = linVel - (angVel * ddmr->wheelBase); // right wheel angular speed rad/s
}


static void robotTankDrive(Robot* ddmr, const float velL, const float velR){
    //@Brief: Drive Mobile Robot in Tank Drive Configuration
    //@Description: Sets Motor Voltage independently.
    //@Param: velL is a % 0-1

    float vL = velL * VBAT_MAX; // convert % to voltage
    float vR = velR * VBAT_MAX;
    motorSetVoltage(&ddmr->motorL, vL); // apply voltage
    motorSetVoltage(&ddmr->motorR, vR); 
}

#endif // ROBOT_H