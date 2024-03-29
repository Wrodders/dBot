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
    wR = Va - (Wa * L 
    wL = Va + Wa * L)
*/
typedef struct State{
    float angularV;
    float linearV;
    float posX, posY;
}State;


typedef struct DDMR{
    Motor motorL, motorR;
    Encoder encL, encR;
    const float wheelR; // R
    const float wheelBase; // L
    
    // kinematic state
    State state;
}DDMR; // Deferential Drive Mobile Robot




static DDMR ddmrInit(void){
    //@Brief: Initializes Peripherals Sensors and Controllers for Low Level Robot Robot
    //@Description: Sets Up Low Level Differential Drive Robot Model with constant speed motors  
    //Constant speed required by kinematics model implemented with PI Controller though Fixed Time Task
    
    DDMR dBot = {
        .wheelBase = WHEEL_BASE, 
        .wheelR = WHEEL_RADIUS,        

        .encL = encoderInit(ENC_L_TIM, UINT16_MAX, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT, ENC_L_AF),
        .motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT),
            
        .encR = encoderInit(ENC_R_TIM, UINT16_MAX, ENC_R_A, ENC_R_PORT, ENC_R_B, ENC_R_PORT, ENC_R_AF),
        .motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT),
        

        };

    motorConfig(&dBot.motorL, &dBot.encL, VBAT_MAX, 0.3f, true, BETA_SPEED);
    motorConfig(&dBot.motorR, &dBot.encR, VBAT_MAX, 0.3f, false,  BETA_SPEED);

    motorDrvEn(&dBot.motorL); // enable DRV8833 & pwm
    motorDrvEn(&dBot.motorR); // enable DRV8833 & pwm

    motorStop(&dBot.motorL);
    motorStop(&dBot.motorR);
    

    return dBot;
}


static void ddmrOdometry(DDMR *dBot){
    //@Brief: Calculate Mobile Robots Kinematic State
    //@Description: xdot = linVel * cos(theta)
    //              ydot = linVel * sin(theta)
    //              thetadot = angVel
    // Integrating these values we can obtain the Mobile Robots Pose q = [x y theta]^T
    // Assuming the constant acceleration.

    //  [linV angV]^T =  R/2 * [1 1; 1/L -1/L]^T * [wR wL]^T
    //  x(t) = (vR + vL)/(vR -vL) * w/2 * sin(t*(vR - VL)/w)
    //  y(t) = (vR + vL)/(vR -vL) * w/2 * cos(t*(vR - VL)/w) + (vR + vL)/(vR - vL) * w/2

    dBot->state.linearV = (dBot->wheelR / 2) * (dBot->motorL.angularVel + dBot->motorR.angularVel);
    dBot->state.angularV = (dBot->wheelR / (2* dBot->wheelBase)) * (dBot->motorL.angularVel - dBot->motorR.angularVel);

  

}

static void ddmrDrive(DDMR* dBot, const float linVel, const float angVel){
    //@Brief: Drive Mobile robot in Differential Drive Configuration
    //@Description: Computes Inverse Kinematics of Robot Model.
    //              

    float wLTarget = (linVel*MPS_TO_RPS) + (angVel * 2 * dBot->wheelBase); // left wheel angular speed rad/s
    float wRTarget = (linVel*MPS_TO_RPS) - (angVel * 2 * dBot->wheelBase); // right wheel angular speed rad/s
    motorSetSpeed(&dBot->motorL,wLTarget);
    motorSetSpeed(&dBot->motorR,wRTarget);
}


static void ddmrTankDrive(DDMR* dBot, const float pwrL, const float pwrR){
    //@Brief: Drive Mobile Robot in Tank Drive Configuration
    //@Description: Wheel Power Sets Motor Voltage Independently 

    float vL = _clamp(pwrL, -1, 1) * VBAT_MAX; // convert % to voltage
    float vR = _clamp(pwrR, -1, 1) * VBAT_MAX;
    motorSetSpeed(&dBot->motorL, vL);
    motorSetSpeed(&dBot->motorR, vR);
}


#endif // DDMR_H