#ifndef DDMR_H
#define DDMR_H

#include "motor.h"

// ******* Differential Drive Mobile Robot ********* // 
struct DDMR{
    const float wheelR;     // R [m]
    const float wheelBase;  // L [m]
    float linAlpha;   // LPF coeff
    float angAlpha;   // LPF coeff 
    // kinematic state
    float linVel;   // m/s    
    float angVel;   // angular vel [rad/s]
    float xpos, ypos, theta; // position and orientation cm 
    float dt;

}DDMR; // Deferential Drive Mobile Robot


static struct DDMR ddmrInit(const float wheelRadius, const float wheelBase, const float linAlpha, const float angAlpha){
    struct DDMR ddmr =  {
        .wheelR = wheelRadius,
        .wheelBase = wheelBase,
        .linAlpha = linAlpha,
        .angAlpha = angAlpha,
        .linVel = linAlpha,
        .dt = VEL_CNTRL_PERIOD*MS_TO_S,
    };
    return ddmr;
} 

//@Brief: Compute Kinematic State of Mobile Robot using WheelOdometry
//@Description:  vel = R * (wR + wL) / 2
//               angVel = 2R * (wR - wL) / L
static void ddmrEstimateOdom(struct DDMR *const ddmr, const struct Motor *const mL, const struct Motor *const mR){
    float vel = (mL->angularVel * RPS_TO_MPS + mR->angularVel * RPS_TO_MPS ) * 0.5f;    // convert to mps 
    ddmr->linVel = (ddmr->linAlpha * vel) + (1.0f - ddmr->linAlpha) * ddmr->linVel;     // lpf filter 
    float angVel = 2*ddmr->wheelR*( mL->angularVel - mR->angularVel) / ddmr->wheelBase; // ang vel in rad/s
    ddmr->angVel = (ddmr->angAlpha * angVel) + (1.0f - ddmr->angAlpha ) * ddmr->angVel; // lpf filter
    ddmr->theta += ddmr->angVel * ddmr->dt; 
    ddmr->theta = fmodf(ddmr->theta, 2*M_PI);// bound to 2pi
    ddmr->xpos += ddmr->linVel*100 * cosf(ddmr->theta) * ddmr->dt;
    ddmr->ypos += ddmr->linVel*100 * sinf(ddmr->theta) * ddmr->dt;
}

#endif // DDMR_H