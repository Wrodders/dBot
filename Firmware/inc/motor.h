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

struct HBridgeDriver{
    uint32_t timPerif;      // PWM Timer Peripheral
    enum tim_oc_id timCH_A; // H Bridge PWM Channels
    enum tim_oc_id timCH_B; // H Bridge PWM Channels
    struct GPIO pwmA;       // H Bridge Pin
    struct GPIO pwmB;       // H Bridge Pin
    struct GPIO en;         // Enable Pin
    float vPSU;             // PSU Voltage   
    float vMin;             // Minimum Voltage to Drive Motor
    struct GPIO mode;       // Mode Pin (DRV8876 only)
}HBridgeDriver;             // PWM H Bridge Motor Driver 

struct Motor {
    struct HBridgeDriver drv;
    int flipDir;            // 1 or -1
    struct Encoder *enc;    // Encoder
    struct PID torqueCtrl;  // Torque PID Controller
    struct PID speedCtrl;   // Shaft Rotation Speed PI Controller
    float wheelRPS;         // Estimated angular speed rps
    float alpha;            // speed lowpass filter parameter
    float maxVel;           // Saturation
    float vMax;             // Maximum Voltage
}Motor;


static struct Motor motorInit(const uint32_t timPerif,const uint32_t pwmPort, 
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
        .speedCtrl = pidInit(-VSYS, VSYS, SPEED_KP, SPEED_KI, SPEED_KD, (WSPEED_CNTRL_PERIOD * MS_TO_S))
    };
    pwmInit(m.drv.timPerif,25000); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_A, m.drv.pwmA.port, m.drv.pwmA.pin, GPIO_AF1); 
	pwmConfig(m.drv.timPerif, m.drv.timCH_B, m.drv.pwmB.port, m.drv.pwmB.pin, GPIO_AF1); 
    pwmStart(m.drv.timPerif);
    timer_enable_break_main_output(m.drv.timPerif); // for advanced timers only

    return m;
}



static void motorConfig(struct Motor *m,struct Encoder* enc, const float vPSU, const float vMin,
                        const float vMax, const int flipDir, float alpha, float maxVel){
    //@Brief: Configs Motor parameters
    m->enc = enc;
    m->drv.vPSU = vPSU;
    m->drv.vMin = vMin; 
    m->flipDir = flipDir == false ? 1 : -1;
    m->alpha = alpha;
    m->maxVel = maxVel;
    m->vMax = vMax;
}

//@Brief: Starts Motor Driver 
static void motorEnable(struct Motor *motor){
    pidClear(&motor->speedCtrl); // clean start
    pidEnable(&motor->speedCtrl); 
    gpio_set(motor->drv.en.port, motor->drv.en.pin); // enable driver
}
// @Brief: Stops Motor Driver
static void motorDisable(struct Motor *motor){
    pidDisable(&motor->speedCtrl);
    gpio_clear(motor->drv.en.port, motor->drv.en.pin); // disable driver board
}

/*
@Brief: Sets the pwm on a Unipolar DC H bridge
@Description: Applies PWM to H bridge according to motor configuration
*/
static void motorSetUnipolarPWM(const struct Motor* motor, const float duty, const int dir){
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


static void motorSetBipolarPWM(const struct Motor* motor, const float duty, const int dir){
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, duty); 
    if(dir == motor->flipDir){
        // Fwds = (1) flipped = (1) ||  BCK = (0) nFlipped = (0) -> PWM B
        // DRIVE BACKWARDS
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0); // drive low
        
    }
    else{
        // Fwds = (1) flipped = (0) ||  BCK = (0) Flipped = (1) -> PWM A
        // DRIVE FORWARDS
        pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 1);
    }
}

/*
@Brief: Sets PWM Duty Cycle as voltage vector
@Description: Applies voltage to H bridge according to motor configuration
*/
static void motorSetVoltage(const struct Motor* motor, const float voltage){
    int dir = _sign(voltage);
    float v = _fabs(voltage);
    v = v <= motor->drv.vMin ? motor->drv.vMin : v; // deadband
    v = v >= motor->vMax ? motor->vMax : v; // saturation
    float duty = _clamp(v / motor->drv.vPSU, 0, 1); // normalize to PSU voltage
    //motorSetUnipolarPWM(motor, duty, dir); // apply to unipolar H bridge
    motorSetBipolarPWM(motor, duty, dir); // apply to bipolar H bridge
}

//@Brief: Sets H Bridge Inputs High
static void motorStop(const struct Motor *motor){ 
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 1);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 1);
}

//@Brief: Sets H Bridge Inputs Low
static void motorBreak(const struct Motor *motor){
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_A, 0);
    pwmSetDuty(motor->drv.timPerif, motor->drv.timCH_B, 0);
}

//@Brief: Sets Target Speed for Motor in RPS
static void motorSetTrgtSpeed(struct Motor *motor, const float trgtRPS){
    float v = _clamp(trgtRPS, -motor->maxVel, motor->maxVel);
    motor->speedCtrl.ref = v;
}

/*
@Brief: Estimates GearBox Shaft Angular Speed in Rotations per Second
@Description: Uses Quadrature Encoder to estimate motor shaft speed
Applies low-pass filter to smooth quantization errors 
*/
static void motorEstSpeed(struct Motor* motor) {
    uint32_t mCount = encoderRead(motor->enc);
    int32_t dCount = (int32_t)(mCount - motor->enc->lastCount); 
    dCount = (dCount + (UINT16_MAX + 1)) % (UINT16_MAX + 1); // wrap around on overflow
    if (dCount > (UINT16_MAX / 2)) { dCount -= (UINT16_MAX + 1);}  // Convert to signed range [-32768, 32767]
    motor->enc->lastCount = mCount; 
    float measuredRPS = (float)dCount * TICKS_TO_RPS * -motor->flipDir * motor->enc->flipDir; // convert to rotations per second at gearbox output
    motor->wheelRPS = iirLPF(motor->alpha, measuredRPS, motor->wheelRPS); 
}



#endif // DCMOTOR_H