#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "motor.h"


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


typedef struct PID{
    float kp,ki,kd;
    float dt; // sample time
    
    float ref; // reference signal
    float lastErr;
    float controlU; 
    float integrator; 
}PID;

typedef struct DDMR{

    Motor motorL;
    Motor motorR;

    PID pidL;
    PID pidR;

    float wheelR; // R
    float wheelBase; // 2L
    
    // Pose
    float angularV;
    float linearV;

    float dt; // speed control update rate, s
}DDMR; // Deferential Drive Mobile Robot



static float pidStep(PID *pid, float err){
    //Apply PID control to error

    // proportional

    pid->integrator = (pid->integrator + pid->lastErr)*pid->dt;
    pid->controlU = (pid->kp * err) + (pid->ki * pid->integrator);
    pid->lastErr = err;

}


static void ddmrDrive(DDMR *ddmr, float angV, float linV){
    // Main Driving Procesess, majse sur eot call this often

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

    // update motor last count
    mL->enc->count = mLCount;
    mR->enc->count = mRCount;

    // apply Speed control PI


    



    
    

}

#endif // KINEMATICS_H