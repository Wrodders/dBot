#ifndef DDMR_H
#define DDMR_H

#include "motor.h"

// 
/*  // ******* Differential Drive Mobile Robot ********* // 

Robot Frame:

x_dot = (v/2) * (w1 + w2) * cos(theta) 
y_dot = (v/2) * (w1 + w2) * sin(theta)
theta_dot = (v/L) * (w1 - w2)

where:
    v = linear velocity
    w1 = angular velocity of left wheel
    w2 = angular velocity of right wheel
    L = wheel base


*/ // ************************************************* //
struct DiffDriveModel{
    const float wheelR;     // R [m]
    const float wheelBase;  // L [m]
    float linearVelAlpha;   // LPF coeff
    float angularVelAlpha;  // LPF coeff 
    // kinematic state
    float linearVel;    // m/s    
    float angularVel;   // angular vel [rad/s]
    float dt;           // sample period [s]
}DiffDriveModel; // Deferential Drive Mobile Robot




static struct DiffDriveModel ddmrInit(const float wheelRadius, const float wheelBase, const float linearVel_alpha, const float angularVel_alpha){
    struct DiffDriveModel ddmr =  {
        .wheelR = wheelRadius,
        .wheelBase = wheelBase,
        .linearVelAlpha = linearVel_alpha,
        .angularVelAlpha = angularVel_alpha,
        .linearVel = 0.0f,
        .angularVel = 0.0f,
        .dt = VEL_CNTRL_PERIOD*MS_TO_S,
    };
    return ddmr;
} 

//@Brief: Compute Inverse Kinematic State of Mobile Robot using Wheel Odometry
static void ddmrEstimateOdom(struct DiffDriveModel *const ddmr, const struct Motor *const motorLeft, const struct Motor *const motorRight){
    float speed =  RPS_TO_MPS * (motorLeft->shaftRPS + motorRight->shaftRPS) * 0.5f;                            // convert to mps 
    ddmr->linearVel = iirLPF(ddmr->linearVelAlpha, speed, ddmr->linearVel);                                      // lpf filter
    float angVel = (motorLeft->shaftRPS - motorRight->shaftRPS) * ddmr->wheelR/ddmr->wheelBase ;                // ang vel in rad/s
    ddmr->angularVel = iirLPF(ddmr->angularVelAlpha, angVel, ddmr->angularVel);                                  // lpf filter
}

#endif // DDMR_H