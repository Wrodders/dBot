#ifndef DDMR_H
#define DDMR_H


#include "motor.h"
#include "pid.h"

// ******* Differential Drive Mobile Robot ********* // 
//@Brief: Controls Robot State in 2D plane
//@Description: Implements Differential Drive Kinematic Model
//              LinVel AngVel Control

/*
    Models and Controls the kinematics of the TWSB Robot in the 2D Plane 
    Provides a DDMR Controllable Robot via cascaded PID 
    Forward Kinematics:
    Va = R * (wR + wL) / 2
    Wa = 2R * (wR - wL) / L
    Inverse Kinematics: 
    wR = (Va - Wa) * L
    wL = (Va + Wa) * L
*/


typedef struct DDMR{
    Motor motorL, motorR;
    Encoder encL, encR;
    const float wheelR; // R
    const float wheelBase; // L
    
    // kinematic state
    float linVel;
    const float aLinVel;  // lpf alpha
  
}DDMR; // Deferential Drive Mobile Robot

static DDMR ddmrInit(void){
    //@Brief: Initializes Peripherals Sensors and Controllers for Low Level Robot Robot
    //@Description: Sets Up Low Level Differential Drive Robot Model with constant speed motors  
    //Constant speed required by kinematics model implemented with PI Controller though Fixed Time Task
    
    DDMR ddmr = {
        .wheelBase = WHEEL_BASE, 
        .wheelR = WHEEL_RADIUS,        

        .encL = encoderInit(ENC_L_TIM, UINT16_MAX, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT, ENC_L_AF),
        .motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT),
            
        .encR = encoderInit(ENC_R_TIM, UINT16_MAX, ENC_R_A, ENC_R_PORT, ENC_R_B, ENC_R_PORT, ENC_R_AF),
        .motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT),
        .aLinVel = VEL_ALPHA,
        };

    motorConfig(&ddmr.motorL, &ddmr.encL, VBAT_MAX, 1.3f, true, SPEED_BETA, RPS_MAX);
    motorConfig(&ddmr.motorR, &ddmr.encR, VBAT_MAX, 1.3f, false,  SPEED_BETA, RPS_MAX);

    motorSetVel(&ddmr.motorL, 0.0f);
    motorSetVel(&ddmr.motorR, 0.0f);

    motorDisable(&ddmr.motorL); // enable DRV8833
    motorDisable(&ddmr.motorR); // enable DRV8833
    return ddmr;
}
static void ddmrOdometry(DDMR *ddmr){
    //@Brief: Compute Kinematic State of Mobile Robot using Odometry
    float mVel = (ddmr->motorL.angularVel * RPS_TO_MPS + ddmr->motorR.angularVel * RPS_TO_MPS ) * 0.5f; // convert to mps 
    ddmr->linVel = (ddmr->aLinVel * mVel) + (1.0f - ddmr->aLinVel) * ddmr->linVel;  // lpf filter 
}

static void ddmrDiffDrive(DDMR* ddmr, float linVel, float angVel){
        //@Brief: Computes Inverse DDMR Kinematics sets Motor Target velocities  
            
        float wLTarget = (linVel*MPS_TO_RPS) + (angVel * 2 * ddmr->wheelBase); //  // RPS
        float wRTarget = (linVel*MPS_TO_RPS) - (angVel * 2 * ddmr->wheelBase); // // RPS
        motorSetVel(&ddmr->motorL,wLTarget); // set target reference speed in rps
        motorSetVel(&ddmr->motorR,wRTarget);
}

static void ddmrTankDrive(DDMR* ddmr, const float pwrL, const float pwrR){
    //@Brief: Drive Mobile Robot in Tank Drive Configuration
    //@Description: Wheel Power Sets Motor Voltage Independently 

    float vL = _clamp(pwrL, -1, 1) * VBAT_MAX; // convert % to voltage
    float vR = _clamp(pwrR, -1, 1) * VBAT_MAX;
    motorSetVel(&ddmr->motorL, vL);
    motorSetVel(&ddmr->motorR, vR);
}

static void ddmrStop(DDMR *ddmr){
    //@Brief: Stops Motors
    motorSetVel(&ddmr->motorL, 0.0f);
    motorSetVel(&ddmr->motorR, 0.0f);
}

static void ddmrEStop(DDMR *ddmr){
    //@Brief: Imitalsy stopsn moters then sets targets to 0.
    motorStop(&ddmr->motorL);
    motorStop(&ddmr->motorR);
    motorSetVel(&ddmr->motorL, 0.0f);
    motorSetVel(&ddmr->motorR, 0.0f);
}



#endif // DDMR_H