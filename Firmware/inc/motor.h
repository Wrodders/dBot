#ifndef DCMOTOR_H
#define DCMOTOR_H

/************************** 
DC MOTOR Configures and uses PWM Timers for DC Motor Voltage Control
NOTE: TIMER PERIF RCC Must be initialized previously. 
Motors Bind To Encoders. 
All velocities are relative to the motors frame. 
DC Motor Transfer Function
wR = wL = V * Kt / (s^2(LaJm) + s(RaJm + BmLa) + KtKe + RaBm)    

**************************/

#include "../common/common.h"
#include "../drivers/pwm.h"
#include "../drivers/encoder.h"
#include "../drivers/gpio.h"

#include "pid.h"

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
    int flipDir; // 1  or -1

    Encoder *enc;
    PID pi;
    float angularVel; // angular speed rad/s
    float alpha; // speed lowpass filter parameter
    int16_t dCount;

    float maxVel;
}Motor;

static Motor motorInit(const uint32_t timPerif, const uint32_t pwmPort, 
                        const enum tim_oc_id timCH_A, const uint32_t pwmA, 
                        const enum tim_oc_id timCH_B, const uint32_t pwmB,  
                        const uint32_t enPin, const uint32_t enPort){
    //@Brief: Inits Motor 
    Motor m = {
        .drv.timPerif = timPerif,
        .drv.timCH_A = timCH_A,
        .drv.timCH_B = timCH_B,
        .drv.pwmA.pin = pwmA,
        .drv.pwmA.port = pwmPort,
        .drv.pwmB.pin = pwmB,
        .drv.pwmB.port = pwmPort,
        .drv.en = initGPIO(enPin, enPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE),
        .pi = pidInit(-VBAT_MAX, VBAT_MAX, SPEED_KP, SPEED_KI, SPEED_KD, (WSPEED_CNTRL_PERIOD * MS_TO_S))
    };

    gpio_clear(m.drv.en.port, m.drv.en.pin); // set off
    // Initialize Timer PWM
    pwmInit(m.drv.timPerif,84,25000); // set freq to 1Hz period to 25000 ARR reg
	pwmConfig(m.drv.timPerif, m.drv.timCH_A, m.drv.pwmA.port, m.drv.pwmA.pin, GPIO_AF1); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_B, m.drv.pwmB.port, m.drv.pwmB.pin, GPIO_AF1); 
    
    pwmStart(m.drv.timPerif);
    timer_enable_break_main_output(m.drv.timPerif); // for advanced timers only

    return m;
}

static void motorConfig(Motor *m,Encoder* enc, const float vPSU, const float vMin,
                        const int flipDir, float alpha, float maxVel){
    //@Brief: Configs Motor parameters
    m->enc = enc;
    m->drv.vPSU = vPSU;
    m->drv.vMin = vMin; 
    m->flipDir = flipDir == false ? 1 : -1;
    m->alpha = alpha;
    m->maxVel = maxVel;
}

static void motorEnable(Motor *motor){
    //@Brief: Starts Motor Driver 
    gpio_set(motor->drv.en.port, motor->drv.en.pin); // enable driver
}

static void motorDisable(Motor *motor){
    //pidDisable(&motor->pi);
    gpio_clear(motor->drv.en.port, motor->drv.en.pin);
}

static void motorSetUnipolar(const Motor* motor, const float duty, const int dir){
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
    //              Backwards == -ve => dir -1
    int dir = _sign(voltage);
    float v = _fabs(voltage);
    v = v <= motor->drv.vMin ? motor->drv.vMin*dir : v;
    float dc = _clamp(v/motor->drv.vPSU, 0, 1.0f); // convert to % of battery
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

static void motorSetTrgtVel(Motor *motor, const float vel){
    //@Brief: Pushes Target speed through speedCurve buffer.
    //@Description: Applies Trapezium Ramp to velocities
    //              Ramps at increments of SPEED_CTRL_PERIOD 
    //              Based on max acceleration value
    float v = _clamp(vel, -motor->maxVel, motor->maxVel);
    motor->pi.ref = v;
}

static void motorCalSpeed(Motor* motor){
    //@Brief: Calculate Angular Speed in Rotations per Second
    //@Description: Applies lowpass filter to smooth Quantization errors 
    uint16_t mCount = encoderRead(motor->enc);
    motor->dCount = mCount - motor->enc->lastCount;
    motor->enc->lastCount = mCount;
    float mSpeed = motor->dCount * TICKS_TO_RPS * motor->flipDir ; // rotations per second
    // Apply Low pass filter to speed measurement  b*speed[n] + (1-b)*speed[n-1] 
    motor->angularVel = (motor->alpha * mSpeed) + (1.00f - motor->alpha) * motor->angularVel;
}

static void motorSpeedCtrl(Motor* motor){
    //@Brief: Regulates Estimated Speed to ref speed
    motorCalSpeed(motor);
    pidRun(&motor->pi, motor->angularVel);
    motorSetVoltage(motor, motor->pi.out);
}


#endif // DCMOTOR_H