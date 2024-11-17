#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../common/common.h"

/* 
Proportional Integral Derivative Algorithm
*/

typedef struct PID{
    float ref;                   // u
    float error;                 // e
    float lastErr;               // ze
    float pTerm,iTerm,dTerm;    // Working Variables
    float kp,ki,kd;             // Preset with dt applied
    const float dt;             // Sample period [s]
    float min, max;             // Saturation 
    float out;                  // Controlled output signal
    bool en;                    // Enable PID Computation   
}PID;

static inline void pidEnable(PID *pid){pid->en = true;}                  
static inline void pidDisable(PID *pid){pid->en = false;}
static inline void pidClear(PID *pid){pid->iTerm=0;} 
static inline void pidSetKp(PID *pid, float kp){pid->kp = kp;}  
static inline void pidSetKi(PID *pid, float ki){pid->ki = ki * pid->dt;} // precompute save calculations 
static inline void pidSetKd(PID *pid, float kd){pid->kd = kd / pid->dt;} //
static inline void pidSetMin(PID* pid, float min){pid->min = min;}
static inline void pidSetMax(PID* pid, float max){pid->min = max;}
static inline void pidSetRef(PID* pid, float ref){pid->ref = ref;} 

static PID pidInit(float min, float max, float kp, float ki, float kd, float deltaT){
    PID pid = {
        .dt = deltaT,
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .min = min,
        .max = max,
        .ref = 0.0f,
        .en = true
    };
    return pid;
}

static float pidRun(PID* pid, const float measurement){
    //@Brief: Steps through PID Algorithm at constant dt
    //@Description: Calculates correction signal from ref measurement error.  
    if(pid->en == false){return 0;}
    pid->error = pid->ref - measurement;                // calculate error
    pid->pTerm = pid->error * pid->kp;                  
    pid->iTerm += pid->error * pid->ki*pid->dt;                  
    pid->iTerm = _clamp(pid->iTerm,pid->min , pid->max); // Apply Integrator Saturation    
    pid->dTerm = (pid->kd/pid->dt)*(pid->error - pid->lastErr);   
    pid->lastErr = pid->error;
    float y = pid->pTerm + pid->iTerm + pid->dTerm;    // calculate correction signal 
    return _clamp(y, pid->min, pid->max);              // Apply output Saturation
}

#endif // CONTROLLER_H