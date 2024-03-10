#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../common/common.h"

typedef struct PID{

    float target;
    float error;
    float lastError;
    float pTerm;
    float iTerm;
    float dTerm; 

    float kp,ki,kd;  // Preset with dt applied

    const float dt;

    float min, max;

    float out;
}PID;

static float pidRun(PID* pid, float measurement){
    //@Brief: Steps through PID Algorithm at constant dt
    //@Description: Calculates correction signal from target measurement error.  

    // calculate error
    pid->error = pid->target - measurement;

    pid->pTerm = pid->error * pid->kp;
    pid->iTerm += pid->error * pid->ki;
    pid->iTerm = _clamp(pid->iTerm, pid->min, pid->max);
    //TODO Implement D

    // calculate correction signal 
    pid->out = pid->pTerm + pid->iTerm + pid->dTerm;
    pid->out = _clamp(pid->out, pid->min, pid->max);

    return pid->out;
}

static void pidSetKp(PID *pid, float kp){pid->kp = kp / pid->dt;}
static void pidSetKi(PID *pid, float ki){pid->ki = ki / pid->dt;}
static void pidSetKd(PID *pid, float kd){pid->kd = kd / pid->dt;}



#endif // CONTROLLER_H