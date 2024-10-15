#ifndef CONTROL_H
#define CONTROL_H

#include "../common/common.h"
#include "pid.h"

// ******* Two Wheel Self-Balancing  Mobile TWSB ********* // 
//@Brief: Controls TWSB Balance
typedef struct Controller{
    PID balanceCtrl;
    float trgtAngVel;
    float thetaOffset;

}Controller; // Deferential Drive Mobile TWSB

static Controller cntrlInit(void){
    //@Brief: Inittialaiizses controllers for Self Balancing 

    Controller cntrl = {
        .balanceCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S)),
        .trgtAngVel = 0.0f,
        .thetaOffset = BAL_OFFSET
    };

    return cntrl;
}

static inline void cntrlTheta(Controller *cntrl, float theta){cntrl->balanceCtrl.ref = theta + cntrl->thetaOffset;}
static inline void cntrlAngVel(Controller *cntrl, float angVel){cntrl->trgtAngVel = angVel;}


#endif // CONTROL_H