#ifndef ROBOT_H
#define ROBOT_H

#include "../common/common.h"
#include "motor.h"
#include "pid.h"


/*
Kinematics Controller for TWSB-DDMR Model of robot. 

DDMR - Differential Driver Mobile Robot 
    Models and Controls the kinematics of the Robot in the 2D Plane 
    Each motor has a PD Speed controller, utilizes Quadrupole Encoder Timer
    
    The model maps Target Linear & Angular Velocity to Motor Velocities
    by the transfer function :

    q = [ x_dot y_dot theta_dot ]^T * [wR wR]^T

    wR = wL = V * Ki/(s^2(LaRa) +s(RaJm + BmLa) + KtKe + RaBm)        
*/  


typedef struct DDMR{
    Motor motorL, motorR;
    Encoder encL, encR;
    PID pidL, pidR;

    const float wheelR; // R
    const float wheelBase; // 2L
    
    // Pose
    float angularV;
    float linearV;
    float posX, posY;

    float dt; // speed control update rate, s
}DDMR; // Deferential Drive Mobile Robot



static DDMR ddmrInit(){
    //@Brief: Initializes Peripherals Sensors and Controllers for Low Level DDMR Robot
    
    DDMR ddmr = {0};

    ddmr.motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_CH1, TIM_OC2, M_L_CH2, DRV_EN_PIN, DRV_EN_PORT);



    return ddmr;
}


static void ddmrDrive(DDMR *ddmr, float angV, float linV){
    // Main Driving Processes

    Motor *mL = &ddmr->motorL;
    Motor *mR = &ddmr->motorR; // calculations

    // Get target Wheel Speeds 
    float wLTarget = linV + (angV * ddmr->wheelBase); // left wheel angular speed rad/s
    float wRTarget = linV - (angV * ddmr->wheelBase); // right wheel angular speed rad/s


    // get the current wheel speeds from encoder;
    uint32_t mLCount =  encoderRead(mL->enc); // measured Left count
    uint32_t mRCount =  encoderRead(mR->enc); // measured Right count
    
    mL->angularSpeed = (float)(mLCount - mL->enc->lastCount) / ddmr->dt;
    mR->angularSpeed = (float)(mLCount - mL->enc->lastCount) / ddmr->dt;

    float errL = wLTarget - mL->angularSpeed; // error is speed
    float errR = wRTarget - mR->angularSpeed;



    // update encoders last count
    ddmr->encL.lastCount = mLCount;
    ddmr->encR.lastCount = mRCount;


}

#endif // ROBOT_H