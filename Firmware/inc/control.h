#ifndef CONTROL_H
#define CONTROL_H

#include "../common/common.h"

#include "imu.h"
#include "pid.h"
#include "ddmr.h"

// ******* Two Wheel Self-Balancing  Mobile TWSB ********* // 
//@Brief: Controls TWSB Balance
typedef struct Controller{
    PID balanceCtrl;
    PID motionCtrl;
    float trgtAngVel;
    float thetaOffset;

}Controller; // Deferential Drive Mobile TWSB

static Controller cntrlInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    Controller bot = {
        .balanceCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S)),
        .motionCtrl = pidInit(-BAL_MAX_RECOVERY, BAL_MAX_RECOVERY, VEL_KP, VEL_KI, VEL_KD, (MCTRL_PERIOD * MS_TO_S)),
        .trgtAngVel = 0.0f,
        .thetaOffset = BAL_OFFSET
    };

    return bot;
}

static inline void cntrlTheta(Controller *cntrl, float theta){cntrl->balanceCtrl.ref = theta - cntrl->thetaOffset;}
static inline void cntrlLinVel(Controller *cntrl, float linVel){cntrl->motionCtrl.ref = linVel;}
static inline void cntrlAngVel(Controller *cntrl, float angVel){cntrl->trgtAngVel = angVel;}


#endif // CONTROL_H