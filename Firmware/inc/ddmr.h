#ifndef DDMR_H
#define DDMR_H

#include "motor.h"

// ******* Differential Drive Mobile Robot ********* // 
typedef struct DDMR{
    const float wheelR;     // R [m]
    const float wheelBase;  // L [m]
    const float linAlpha;   // LPF coeff
    const float angAlpha;   // LPF coeff 
    // kinematic state
    float linVel;   // m/s    
    float angVel;   // angular vel [rad/s]
}DDMR; // Deferential Drive Mobile Robot


static DDMR ddmrInit(const float wheelRadius, const float wheelBase, const float linAlpha, const float angAlpha){
    DDMR ddmr =  {
        .wheelR = wheelRadius,
        .wheelBase = wheelBase,
        .linAlpha = angAlpha,
        .linVel = linAlpha
    };
    return ddmr;
} 

//@Brief: Compute Kinematic State of Mobile Robot using WheelOdometry
//@Description:  vel = R * (wR + wL) / 2
//               angVel = 2R * (wR - wL) / L
static void ddmrEstimateOdom(DDMR *const ddmr, const Motor *const mL, const Motor *const mR){
    float vel = (mL->angularVel * RPS_TO_MPS + mR->angularVel * RPS_TO_MPS ) * 0.5f; // convert to mps 
    ddmr->linVel = (ddmr->linAlpha * vel) + (1.0f - ddmr->linAlpha) * ddmr->linVel;  // lpf filter 
    vel = 2*(ddmr->wheelR) * (mL->angularVel - mR->angularVel) *RPS_TO_RADS;         // ang vel in rad/s
    ddmr->angVel = (ddmr->angAlpha * vel) + (1.0f - ddmr->angAlpha) * ddmr->angVel;  // lpf filter 
}

#endif // DDMR_H