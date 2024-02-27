#ifndef ENCODER_H
#define ENCODEr_H


#include "../common/common.h"
#include <libopencm3/stm32/timer.h>

/*
Hall effect encoder 12 CPR

*/

#define PI  3.14159
#define TIM_MODE_ENC 0x3

typedef struct Encoder {
    GPIO chA;
    GPIO chB;
    uint32_t timPerif;
    uint32_t cpr; // counts per revolution
    
    uint32_t lastCount; // encoder count
}Encoder;

static Encoder encoderInit(uint32_t timPerif, uint32_t pinA, uint32_t portA, uint32_t pinB, uint32_t portB, uint16_t cpr ){
    //@Brief: Sets up timer in encoder mode  
    Encoder e = {0};
    e.chA = initGPIO(pinA, portA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP);
    e.chB = initGPIO(pinB, portB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP);
    e.timPerif = timPerif;
    timer_set_period(e.timPerif, cpr - 1);
    timer_slave_set_mode(e.timPerif, TIM_MODE_ENC);
    timer_ic_set_input(e.timPerif,  TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(e.timPerif,  TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(e.timPerif);

    return e;
}

static inline uint32_t encoderRead(Encoder *enc){
    return timer_get_counter(enc->timPerif);
}




#endif // ENCODER_H