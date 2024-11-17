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

struct Driver{
    uint32_t timPerif;
    enum tim_oc_id timCH_A;
    enum tim_oc_id timCH_B;
    struct GPIO pwmA;
    struct GPIO pwmB;
    struct GPIO en;
    float vPSU;
    float vMin; 
}Driver; // PWM H Bridge Motor Driver 

struct Motor {
    struct Driver drv;
    int flipDir;       // 1 or -1
    struct Encoder *enc;      // Encoder
    struct PID tCtrl;         // Torque PID Controller
    struct PID wCtrl;         // Speed PI Controller
    float angularVel;  // angular speed rps
    float alpha;       // speed lowpass filter parameter
    float maxVel;      // Saturation
}Motor;

static struct Motor motorInit(const uint32_t timPerif, const uint32_t pwmPort, 
                        const enum tim_oc_id timCH_A, const uint32_t pwmA, 
                        const enum tim_oc_id timCH_B, const uint32_t pwmB,  
                        const uint32_t enPin, const uint32_t enPort){
    //@Brief: Inits Motor PWM Timers 
    struct Motor m = {
        .drv.timPerif = timPerif,
        .drv.timCH_A = timCH_A,
        .drv.timCH_B = timCH_B,
        .drv.pwmA.pin = pwmA,
        .drv.pwmA.port = pwmPort,
        .drv.pwmB.pin = pwmB,
        .drv.pwmB.port = pwmPort,
        .drv.en = initGPIO(enPin, enPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE),
        .wCtrl = pidInit(-VBAT_MIN, VBAT_MIN, SPEED_KP, SPEED_KI, SPEED_KD, (WSPEED_CNTRL_PERIOD * MS_TO_S))
    };

    // Initialize Timer PWM
    pwmInit(m.drv.timPerif,84,25000); // set freq to 1Hz period to 25000 ARR reg
	pwmConfig(m.drv.timPerif, m.drv.timCH_A, m.drv.pwmA.port, m.drv.pwmA.pin, GPIO_AF1); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_B, m.drv.pwmB.port, m.drv.pwmB.pin, GPIO_AF1); 
    pwmStart(m.drv.timPerif);
    timer_enable_break_main_output(m.drv.timPerif); // for advanced timers only

    return m;
}

static void motorConfig(struct Motor *m,struct Encoder* enc, const float vPSU, const float vMin,
                        const int flipDir, float alpha, float maxVel){
    //@Brief: Configs Motor parameters
    m->enc = enc;
    m->drv.vPSU = vPSU;
    m->drv.vMin = vMin; 
    m->flipDir = flipDir == false ? 1 : -1;
    m->alpha = alpha;
    m->maxVel = maxVel;
}

static void motorEnable(struct Motor *motor){
    //@Brief: Starts Motor Driver 
    pidClear(&motor->wCtrl);
    pidEnable(&motor->wCtrl);
    gpio_set(motor->drv.en.port, motor->drv.en.pin); // enable driver
}
static void motorDisable(struct Motor *motor){
    pidDisable(&motor->wCtrl);
    gpio_clear(motor->drv.en.port, motor->drv.en.pin);
}

static void motorSetUnipolarPWM(const struct Motor* motor, const float duty, const int dir){
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
static void motorSetVoltage(const struct Motor* motor, const float voltage){
    //@Brief: Sets PWM Duty Cycle as voltage vector
    //@Description: Forwards == +ve => dir 1
    //              Backwards == -ve => dir -1
    int dir = _sign(voltage);
    float v = _fabs(voltage);
    v = v <= motor->drv.vMin ? motor->drv.vMin*dir : v;
    float dc = _clamp(v/motor->drv.vPSU, 0, 1.0f); // convert to % of battery
    motorSetUnipolarPWM(motor, dc, dir); // apply to unipolar H bridge
}

static void motorStop(const struct Motor *motor){ 
    //@Brief: Sets H Bridge Inputs High
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 1);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 1);
}

static void motorBreak(const struct Motor *motor){
    //@Brief: Sets H Bridge Inputs Low
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0);
}

static void motorSetTrgtSpeed(struct Motor *motor, const float vel){
    //@Brief: Pushes Target speed through speedCurve buffer.
    float v = _clamp(vel, -motor->maxVel, motor->maxVel);
    motor->wCtrl.ref = v;
}

static void motorEstSpeed(struct Motor* motor) {
    //@Brief: Calculate Angular Speed in Rotations per Second
    //@Description: Applies low-pass filter to smooth quantization errors 
    //@Note: Speed is relative to motor shaft (after gearbox)
    uint32_t mCount = encoderRead(motor->enc);
    int32_t dCount = (int32_t)(mCount - motor->enc->lastCount);
    // Handle overflow and underflow
    dCount = (dCount + (UINT16_MAX + 1)) % (UINT16_MAX + 1);
    // Convert to signed range [-32768, 32767]
    if (dCount > (UINT16_MAX / 2)) { dCount -= (UINT16_MAX + 1);}
    motor->enc->lastCount = mCount;
    float mSpeed = (float)dCount * TICKS_TO_RPS * motor->flipDir;
    motor->angularVel = (motor->alpha * mSpeed) + (1.0f - motor->alpha) * motor->angularVel;
}

static void motorSpeedCtrl(struct Motor *m){
    // Set Motor Voltage based on Speed Control PID 
    // Uses Encoders to Estimate Motor Shaft Speed  
    motorEstSpeed(m);
    m->wCtrl.out = pidRun(&m->wCtrl, m->angularVel);
    motorSetVoltage(m, m->wCtrl.out);
}

#endif // DCMOTOR_H