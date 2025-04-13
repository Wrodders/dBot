#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../common/common.h"

struct PID{
    float ref;                   // u
    float error;                 // e
    float lastErr;               // ze
    float pTerm,iTerm,dTerm;    // Working Variables
    float kp,ki,kd;             // Preset with dt applied
    const float dt;             // Sample period [s]
    const float min;
    const float max;            // Saturation 
    float out;                  // Controlled output signal
    bool en;                    // Enable PID Computation   
}PID;

static inline void pidEnable(struct PID *pid){pid->en = true;}                  
static inline void pidDisable(struct PID *pid){pid->en = false;}
static inline void pidClear(struct PID *pid){pid->iTerm=0; pid->lastErr=0; pid->out=0; pid->ref=0; pid->error=0;}  
static inline void pidSetRef(struct PID* pid, float ref){pid->ref = ref;} 

static struct PID pidInit(float min, float max, float kp, float ki, float kd, float deltaT){
    struct PID pid = {
        .dt = deltaT,
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .min = min,
        .max = max,
        .ref = 0.0f,
        .en = true,
        .out  = 0.0f
    };
    return pid;
}

//@Brief: Steps through PID Algorithm at constant dt
//@Description: Calculates correction signal from ref measurement error.  
static float pidRun(struct PID* pid, const float measurement){

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