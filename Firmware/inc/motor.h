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
#include "../drivers/gpio.h"


typedef struct Driver{
    uint32_t timPerif;
    enum tim_oc_id timCH_A;
    enum tim_oc_id timCH_B;
    GPIO pwmA;
    GPIO pwmB;
    GPIO en;
    float vPSU;
    float vMin; 
}Driver; // Motor Driver DRV8833

typedef struct Motor {
    Driver drv;
    bool flipDir; 
    float angularSpeed; // angular speed rad/s
    float beta; // speed lowpass filter parameter
}Motor;

static Motor motorInit(const uint32_t timPerif, const uint32_t pwmPort, 
                        const enum tim_oc_id timCH_A, const uint32_t pwmA, 
                        const enum tim_oc_id timCH_B, const uint32_t pwmB,  
                        const uint32_t enPin, const uint32_t enPort){
    //@Brief: Inits Motor PWM
    Motor m;
    m.drv.timPerif = timPerif;
    m.drv.timCH_A = timCH_A;
    m.drv.timCH_B = timCH_B;
    m.drv.pwmA.pin = pwmA;
    m.drv.pwmA.port = pwmPort;
    m.drv.pwmB.pin = pwmB;
    m.drv.pwmB.port = pwmPort;


    m.drv.en = initGPIO(enPin, enPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
    gpio_clear(m.drv.en.port, m.drv.en.pin); // set off
    // Initialize Timer PWM
    pwmInit(m.drv.timPerif,84,25000); // set freq to 1Hz period to 25000 ARR reg
	pwmConfig(m.drv.timPerif, m.drv.timCH_A, m.drv.pwmA.port, m.drv.pwmA.pin, GPIO_AF1); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_B, m.drv.pwmB.port, m.drv.pwmB.pin, GPIO_AF1); 
    
    pwmStart(m.drv.timPerif);
    timer_enable_break_main_output(m.drv.timPerif); // for advanced timers only

    return m;
}

static void motorConfig(Motor *m,const float vPSU, const float vMin,
                        const bool flipDir, float beta){
    
    //@Brief: Configs Motor parameters
    m->drv.vPSU = vPSU;
    m->drv.vMin = vMin;
    m->flipDir = flipDir;
    m->beta = beta;
}


static void driverEnable(const Driver *drv){
    //@Brief: Starts Motor Driver PWM
    gpio_set(drv->en.port, drv->en.pin); // enable driver
}

static void motorSetUnipolar(const Motor* motor, const float duty, const bool dir){
    //@Brief: Sets the pwm on a Unipolar DC H bridge:
   
    if(dir == motor->flipDir){
        // Fwds = (1) flipped = (1) ||  BCK = (0) nFlipped = (0) -> PWM B
        // DRIVE BACKWARDS
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0); // drive low
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, duty);  
    }
    else{
        // Fwds = (1) flipped = (0) ||  BCK = (0) Flipped = (1) -> PWM A
        // DRIVE FORWARDS
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, duty);
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0); 
    }
}

static void motorSetVoltage(const Motor* motor, const float voltage){
    //@Brief: Sets PWM Duty Cycle as voltage vector
    //@Description: Forwards == +ve => dir 1
    //              Backwards == -ve => dir 0
    bool dir;
    float v = voltage;
    float vAbs =  _fabs(v);
    if(vAbs < motor->drv.vMin){v = 0;} // limit minium voltage
    if(v >= 0.0f){ dir = 1;}
    else{dir = 0;}
    float dc = _clamp(vAbs/motor->drv.vPSU, 0, 1); // convert to % of battery
    motorSetUnipolar(motor, dc, dir); // apply to unipolar H bridge
}

static void motorStop(const Motor *motor){ 
    //@Brief: Sets H Bridge Inputs High
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 1);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 1);
}

static void motorBreak(const Motor *motor){
    //@Brief: Sets H Bridge Inputs Low
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0);
}

static void motorCalSpeed(Motor* motor, Encoder* enc){
    //@Brief: Calculate Angular Speed in Rotations per Second
    //@Description: Applies lowpass filter to smooth Quantization errors 
    uint16_t mCount = encoderRead(enc);
    int16_t dCount = mCount - enc->lastCount;
    enc->lastCount = mCount;
    float mSpeed = dCount * TICKS_TO_RPS; // rotations per second
    // Apply Low pass filter to speed measurement (1 - b)speed[n] + b*speed[n-1] 
    motor->angularSpeed = (motor->beta * mSpeed) + (1.00f - motor->beta) * motor->angularSpeed;
}

#endif // DCMOTOR_H