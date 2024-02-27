#ifndef DCMOTOR_H
#define DCMOTOR_H


/************************** 
DC MOTOR Configures and uses PWM Timers for DC Motor Voltage Control
NOTE: TIMER PERIF RCC Must be initialized previously. 
Motors Bind To Encoders. 
All velocities are relative to the motors frame. 
**************************/

#include "../common/common.h"
#include "../drivers/pwm.h"
#include "../drivers/encoder.h"


typedef struct Driver{
    uint32_t timPerif;
    enum tim_oc_id timCH_A;
    enum tim_oc_id timCH_B;
    GPIO pwmA;
    GPIO pwmB;
    GPIO en;
    float vPSU;
    float vLimit;
    float vMin; 
}Driver; // Motor Driver DRV8833

typedef struct Motor {
    Driver drv;
    Encoder *enc; 
    bool flipDir; 
    float angularSpeed; // angular speed rad/s
}Motor;

static Motor motorInit(uint32_t timPerif,  uint32_t pwmPort, enum tim_oc_id timCH_A, uint32_t pwmA, enum tim_oc_id timCH_B, uint32_t pwmB,  uint32_t enPin, uint32_t enPort){
    //@Brief: Inits Motor PWM
    Motor m;
    m.drv.timPerif = timPerif;
    m.drv.timCH_A = timCH_A;
    m.drv.timCH_B = timCH_B;
    m.drv.pwmA.pin = pwmA;
    m.drv.pwmA.port = pwmPort;
    m.drv.pwmB.pin = pwmB;
    m.drv.pwmB.port = pwmPort;
    m.drv.en.pin = enPin;
    m.drv.en.port = enPort;

    // Initialize Timer PWM
    pwmInit(m.drv.timPerif,84,1); // set freq to 1Hz period to 25000 ARR reg
	pwmConfig(m.drv.timPerif, m.drv.timCH_A, m.drv.pwmA.port, m.drv.pwmA.pin, GPIO_AF1); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_B, m.drv.pwmB.port, m.drv.pwmB.pin, GPIO_AF1); 
   
    return m;
}

static void motorConfig(Motor *m, Encoder *enc,float vPSU, float vLimit, float vMin, bool flipDir){
    //@Brief: Configs Motor parameters
    m->enc = enc;
    m->drv.vPSU = vPSU;
    m->drv.vLimit = vLimit < vPSU ? vLimit : vPSU ; // vLimit must be below or == vPSU
    m->drv.vMin = vMin;
    m->flipDir = flipDir;
    return; 
}


static void driverEnable(Driver *drv){
    //@Brief: Starts Motor Driver PWM
    pwmStart(drv->timPerif);
    timer_enable_break_main_output(drv->timPerif); // for advanced timers only
    return;
}


static void motorSetVoltage(Motor *motor, float v){
    //@Brief: Sets PWM Duty Cycle as voltage vector
    // Assumes 
    // PWM A High == Forwards
    // PWM B High == Backwards 
    // XOR
    // Forward Spin(1) & motor flipped(1) -> PWM B
    bool dir;
    float vAbs =  _fabs(v);
    if(vAbs < motor->drv.vMin){
       v = 0;
    }
    if(v >= 0.0f){ 
        dir = 1;
    }
    else{
        dir = 0; 
        v = -v;
    }

    v = _clamp(v, 0.0f, motor->drv.vLimit);
    v = _clamp(v/motor->drv.vPSU, 0.0f, 1.0f);
    
    if(dir == motor->flipDir){
        // Fwds = (1) flipped = (1) ||  BCK = (0) nFlipped = (0) -> PWM B
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0); // drive low
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, v); 
    }
    else{
        // Fwds = (1) flipped = (0) ||  BCK = (0) Flipped = (1) -> PWM A
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, v);
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0); 
    }
    return;
}

static void motorStop(Motor *motor){ 
    //@Brief: Sets H Brige Inputs High
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 1);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 1);
    return;
}

static void motorBreak(Motor *motor){
    //@Brief: Sets H Bridge Inputs Low
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0);
    return;
}

static void motorSetVelocity(Motor *motor, float vel){
    //@Brief: Sets Motors Speed Control Reference
    //@Description: DC Motor Transfer Function Z Transform 

    return;

}


#endif // DCMOTOR_H