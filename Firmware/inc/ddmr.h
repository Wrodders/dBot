#ifndef DDMR_H
#define DDMR_H

#include "motor.h"

// 
  // ******* Differential Drive Mobile Robot ********* // 

struct DiffDriveModel{
    const float wheelR;     // R [m]
    const float wheelBase;  // L [m]
    float linearVelAlpha;   // LPF coeff estimate
    float angularVelAlpha;  // LPF coeff estimate

    float linVelFiltAlpha;  // LPF coeff reference
    float angVelFiltAlpha;  // LPF coeff reference
    // kinematic state
    float linearVel;    // m/s    
    float angularVel;   // angular vel [rad/s]
    float posX;         // x position [m]
}DiffDriveModel; // Deferential Drive Mobile Robot




static struct DiffDriveModel ddmrInit(const float wheelRadius, const float wheelBase, 
                                      const float linearVel_alpha, const float angularVel_alpha){
    struct DiffDriveModel ddmr =  {
        .wheelR = wheelRadius,
        .wheelBase = wheelBase,
        .linearVelAlpha = linearVel_alpha,
        .angularVelAlpha = angularVel_alpha,
        .linearVel = 0.0f,
        .angularVel = 0.0f
    };
    return ddmr;
} 

//@Brief: Compute Inverse Kinematic State of Mobile Robot using Wheel Odometry
static void ddmrEstimateOdom(struct DiffDriveModel *const ddmr, const struct Motor *const motorLeft, const struct Motor *const motorRight){
    float speed =  RPS_TO_MPS * (motorLeft->shaftRPS + motorRight->shaftRPS) * 0.5f;                            // convert to mps 
    ddmr->linearVel = iirLPF(ddmr->linearVelAlpha, speed, ddmr->linearVel);                                      // lpf filter
    float angVel = (motorLeft->shaftRPS - motorRight->shaftRPS) * ddmr->wheelR/ddmr->wheelBase ;                // ang vel in rad/s
    ddmr->angularVel = iirLPF(ddmr->angularVelAlpha, angVel, ddmr->angularVel);                                  // lpf filter
    ddmr->posX += ddmr->linearVel * (LQR_CNTRL_PERIOD * MS_TO_S); // update position
}

#endif // DDMR_H