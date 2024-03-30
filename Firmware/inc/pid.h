#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../common/common.h"

typedef struct PID{
    float ref;
    float error;
    float lastErr;
    float pTerm,iTerm,dTerm; 
    float kp,ki,kd;  // Preset with dt applied
    const float dt;
    float min, max;
    float out;
}PID;

static PID pidInit(float min, float max, float kp, float ki, float kd, float deltaT){
    PID pid = {.dt = deltaT};
    pid.kp = kp;
    pid.ki  = ki*deltaT;
    pid.kd = kd/deltaT;
    pid.min = min;
    pid.max = max;
    pid.ref = 0.0f;
    return pid;
}

static void pidRun(PID* pid, float measurement){
    //@Brief: Steps through PID Algorithm at constant dt
    //@Description: Calculates correction signal from ref measurement error.  

    // calculate error
    pid->error = pid->ref - measurement;

    pid->pTerm = pid->error * pid->kp;
    pid->iTerm += pid->error * pid->ki;
    pid->iTerm = _clamp(pid->iTerm, pid->min, pid->max);
    pid->dTerm = pid->kd*(pid->error - pid->lastErr);
    pid->lastErr = pid->error;

    // calculate correction signal 
    pid->out = pid->pTerm + pid->iTerm + pid->dTerm;
    pid->out = _clamp(pid->out, pid->min, pid->max);
}

static inline void pidSetKp(PID *pid, float kp){pid->kp = kp / pid->dt;}
static inline void pidSetKi(PID *pid, float ki){pid->ki = ki / pid->dt;}
static inline void pidSetKd(PID *pid, float kd){pid->kd = kd / pid->dt;}
static inline void pidSetMin(PID* pid, float min){pid->min = min;}
static inline void pidSetMax(PID* pid, float max){pid->min = max;}
static inline void pidSetRef(PID* pid, float ref){pid->ref = ref;}


#endif // CONTROLLER_H