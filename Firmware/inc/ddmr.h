#ifndef DDMR_H
#define DDMR_H


#include "motor.h"


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
    const float wheelR; // R
    const float wheelBase; // L
    // kinematic state
    float linVel;
    float linX; 
    float dt;

}DDMR; // Deferential Drive Mobile Robot



static void ddmrOdometry(DDMR *ddmr, Motor *mL, Motor *mR){
    //@Brief: Compute Kinematic State of Mobile Robot using Odometry
    float mVel = (mL->angularVel * RPS_TO_MPS + mR->angularVel * RPS_TO_MPS ) * 0.5f; // convert to mps 
    ddmr->linVel = (VEL_ALPHA * mVel) + (1.0f - VEL_ALPHA) * ddmr->linVel;  // lpf filter 
    float dx  = ddmr->linVel * (ddmr->dt) ;
    ddmr->linX += dx; 
}






#endif // DDMR_H