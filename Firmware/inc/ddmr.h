#ifndef DDMR_H
#define DDMR_H

#include "motor.h"

// ******* Differential Drive Mobile Robot ********* // 
struct DiffDriveModel{
    const float wheelR;     // R [m]
    const float wheelBase;  // L [m]
    float linearVelAlpha;   // LPF coeff 
    float angularVelAlpha;  // LPF coeff 
    float dt;              // sample time [s]

    float gyroOdoThreshold; // threshold for gyro odometry

    // kinematic state
    float v_b;   // linear velocity m/s    
    float posX;  // x position [m] in robot frame
    float w_b;   // angular vel [rad/s]
   
    float x_g;   // x position [m] in global frame
    float y_g;   // y position [m] in global frame
    float psi; // orientation [rad] in global frame
}DiffDriveModel; // Deferential Drive Mobile Robot

static inline void ddmrReset(struct DiffDriveModel *const ddmr){
    ddmr->v_b = 0.0f;
    ddmr->w_b = 0.0f;
    ddmr->posX = 0.0f;
    ddmr->x_g = 0.0f;
    ddmr->y_g = 0.0f;
    ddmr->psi = 0.0f;
}
static struct DiffDriveModel ddmrInit(const float wheelRadius, const float wheelBase, 
                            const float linearVel_alpha, const float angularVel_alpha, 
                            const float dt){

    struct DiffDriveModel ddmr =  {
        .wheelR = wheelRadius,
        .wheelBase = wheelBase,
        .linearVelAlpha = linearVel_alpha,
        .angularVelAlpha = angularVel_alpha,
        .dt = dt
    };
    return ddmr;
} 


//@Brief: Compute Inverse Kinematic State of Mobile Robot using Wheel Odometry
static void ddmrEstimateOdom(struct DiffDriveModel *const ddmr, const struct Motor *const motorLeft, const struct Motor *const motorRight){
    float speed =  RPS_TO_MPS * (motorLeft->wheelRPS + motorRight->wheelRPS) * 0.5f;             // convert to mps 
    ddmr->v_b = iir_lpf(ddmr->linearVelAlpha, speed, ddmr->v_b);                               // lpf filter
    float delta_psi = (motorLeft->wheelRPS - motorRight->wheelRPS) * ddmr->wheelR/ddmr->wheelBase ; // ang vel in RPS
    ddmr->w_b = iir_lpf(ddmr->angularVelAlpha, delta_psi, ddmr->w_b);                             // lpf filter
    ddmr->posX += ddmr->v_b * ddmr->dt; // update position

    // Compute incremental rotation from odometry measurement
    ddmr->x_g += ddmr->v_b * ddmr->dt * cos(ddmr->psi);
    ddmr->y_g += ddmr->v_b * ddmr->dt * sin(ddmr->psi);
    float heading = ddmr->w_b* 2*M_PI * ddmr->dt; // heading in radians
    // wrap to [-pi, pi]
    ddmr->psi += heading; // update heading
    if (ddmr->psi > M_PI) {
        ddmr->psi -= 2 * M_PI;
    } else if (ddmr->psi < -M_PI) {
        ddmr->psi += 2 * M_PI;
    }
}

static void ddmrGyroOdometry(struct DiffDriveModel *const ddmr,
                            const struct Motor *const motorLeft,
                            const struct Motor *const motorRight,
                            float gyroZ){
    // --- Odometry Update ---
    float wheelSpeed = RPS_TO_MPS * (motorLeft->wheelRPS + motorRight->wheelRPS) * 0.5f;
    ddmr->v_b = iir_lpf(ddmr->linearVelAlpha, wheelSpeed, ddmr->v_b);
    float wheelAngVel = (motorLeft->wheelRPS - motorRight->wheelRPS) * (2*M_PI*ddmr->wheelR) / ddmr->wheelBase;
    float deltaThetaOdo = wheelAngVel * ddmr->dt;
    // bind to [-pi, pi]
    // Compute incremental rotation from gyro measurement
    float deltaThetaGyro = gyroZ * ddmr->dt;
    // Compute the absolute difference between gyro and odometry increments
    float deltaDiff = fabs(deltaThetaGyro - deltaThetaOdo);
    // Decide which measurement to trust based on the threshold:
    if (deltaDiff > ddmr->gyroOdoThreshold) {
    // Catastrophic odometry error detected: use gyro measurement.
    ddmr->psi += deltaThetaGyro;
    ddmr->w_b = gyroZ;
    } else {
    // Normal condition: use odometry measurement.
    ddmr->psi += deltaThetaOdo;
    ddmr->w_b = wheelAngVel;
    }
    ddmr->w_b = iir_lpf(ddmr->angularVelAlpha, ddmr->w_b, ddmr->w_b);
    ddmr->x_g += ddmr->v_b * ddmr->dt * cos(ddmr->psi);
    ddmr->y_g += ddmr->v_b * ddmr->dt * sin(ddmr->psi);
}

#endif // DDMR_H