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
        .balanceCtrl = pidInit(-VEL_MAX, VEL_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S)),
        .motionCtrl = pidInit(-VBAT_MAX, VEL_MAX, VEL_KP, VEL_KI, VEL_KD, (CTRL_PERIOD * MS_TO_S)),
        .trgtAngVel = 0.0f,
        .thetaOffset = BAL_THETA
    };

    return bot;
}

static inline void cntrlLinVel(Controller *bot, float linVel){bot->motionCtrl.ref = linVel;}
static inline void cntrlAngVel(Controller *bot, float angVel){bot->trgtAngVel = angVel;}


#endif // CONTROL_H