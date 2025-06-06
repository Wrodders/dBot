#ifndef PWM_H
#define PWM_H

#include "../common/common.h"
#include <libopencm3/stm32/timer.h>

static void pwmInit(uint32_t timPerif){
    //@Brief: Initializes a Timer peripheral for PWM mode
    timer_set_mode(timPerif,
                    TIM_CR1_CKD_CK_INT,  // No Clock Division
                    TIM_CR1_CMS_EDGE,    // Edge Aligned Mode
                    TIM_CR1_DIR_UP);     // Count Up
    
   
    // General purpose timers APB1 clock domain is 42MHz
    // DS10314 Rev8 6.3.18 Tim timer characteristics
    // TimXCLK = HCLK if APB Prescaler is 1, 2 
    timer_set_prescaler(timPerif,84-1); // 
    timer_set_period(timPerif, 1000000/20000); //
    timer_continuous_mode(timPerif);
}


static void pwmConfig(uint32_t timPerif,enum tim_oc_id timCH, 
    uint32_t gpioPort, uint32_t gpioPin, uint32_t gpioAF){
    timer_disable_oc_output(timPerif, timCH );
    timer_set_oc_mode(timPerif, timCH, TIM_OCM_PWM1); // Output Compare PWM Mode 1
    timer_set_oc_value(timPerif, timCH, 0); // preset to 0
    gpio_mode_setup(gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, gpioPin); 
    gpio_set_af(gpioPort, gpioAF, gpioPin); // Must match to specified timer channel
    gpio_set_output_options(gpioPort, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpioPin);
    timer_enable_oc_output(timPerif,timCH); // enable output Output Compare pin
}

static void pwmSetDuty(uint32_t timPerif, enum tim_oc_id timCH, float dutyCycle){
    //@Brief: Sets Duty Cycle of PWM Output Chanel
    //The duty cycle value is a percentage of the reload register value (ARR)
    const float newCCR = (float)50 * dutyCycle;
    timer_set_oc_value(timPerif, timCH, (uint32_t)newCCR);
}

static void pwmStart(uint32_t timPerif){
    //@Brief: Start PWM Generation
    timer_generate_event(timPerif, TIM_EGR_UG);
    timer_enable_counter(timPerif);
}
#endif // PWM_H