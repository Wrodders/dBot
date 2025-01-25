#ifndef ENCODER_H
#define ENCODEr_H


#include "../common/common.h"
#include <libopencm3/stm32/timer.h>
#include "gpio.h"

#define TIM_MODE_ENC 0x3

struct Encoder {
    struct GPIO chA;
    struct GPIO chB;
    uint32_t timPerif;
    uint16_t period; // counts per revolution
    
    uint32_t lastCount; // encoder count
}Encoder;

 static struct Encoder encoderInit(uint32_t timPerif,  uint32_t period, uint32_t pinA, uint32_t portA, uint32_t pinB, uint32_t portB, uint32_t alternateFunction) {
    struct Encoder e;
    // Store the timer peripheral in the encoder structure
    e.timPerif = timPerif;
    e.lastCount = 0;
    // Configure pins for alternate function mode
    gpio_mode_setup(portA, GPIO_MODE_AF, GPIO_PUPD_NONE, pinA);
    gpio_set_af(portA, alternateFunction, pinA);
    gpio_mode_setup(portB, GPIO_MODE_AF, GPIO_PUPD_NONE, pinB);
    gpio_set_af(portB, alternateFunction, pinB);

    // Configure timer in encoder mode
    timer_set_mode(e.timPerif, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(e.timPerif, 0);
    timer_disable_preload(e.timPerif);
    timer_continuous_mode(e.timPerif);
    timer_set_period(e.timPerif, period - 1);
    timer_slave_set_mode(e.timPerif, TIM_SMCR_SMS_EM3); // Encoder mode 3
    timer_ic_set_input(e.timPerif, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(e.timPerif, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(e.timPerif);
    TIM_CNT(e.timPerif) = 0x0000; // reset counter

    return e;
}

static inline uint32_t encoderRead(struct Encoder *enc){
    return timer_get_counter(enc->timPerif);
}

#endif // ENCODER_H