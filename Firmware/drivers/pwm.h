#ifndef PWM_H
#define PWM_H

#include "../common/common.h"
#include <libopencm3/stm32/timer.h>


static void pwm_config_timer(uint32_t timPerif, uint16_t prescaler, uint32_t freq){
    //@Breif: Initializes a Timer Perfieral for PWM mode
    
    timer_set_mode(timPerif,TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,TIM_CR1_DIR_UP);
    timer_set_prescaler(timPerif, prescaler - 1); 
    timer_set_repetition_counter(timPerif, 0);
    timer_continuous_mode(timPerif);
    uint32_t period = (rcc_ahb_frequency)/ freq;
    timer_set_period(timPerif, period - 1); 
    return;
}

static void pwm_init_output_channel(uint32_t timPerif,enum tim_oc_id timCH, uint32_t gpioPort, uint32_t gpioPin, uint32_t gpioAF){
    //@Breif: Set Up GPIO and Output Compare for PWM
    timer_disable_oc_output(timPerif, timCH );
    timer_set_oc_mode(timPerif, timCH, TIM_OCM_PWM1); // Output Compare PWM Mode 1
    //timer_set_oc_value(timPerif, timCH, 0); // preset to 0
    gpio_mode_setup(gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, gpioPin); 
    gpio_set_af(gpioPort, gpioAF, gpioPin); // Must match to spcified timer channel
     
    timer_enable_oc_output(timPerif,timCH); // enable output Output Compare pin
    timer_enable_preload(timPerif);
    return;
}

static void pwm_set_dutyCycle(uint32_t timPerif, enum tim_oc_id timCH, float dutyCycle){
    //@Breif: Sets Duty Cycle of PWM Output Chanel
    //The duty cycle value is a percentage of the reload register value (ARR)
    const float newCCR = (float)(TIM_ARR(timPerif) * dutyCycle);
    timer_set_oc_value(timPerif, timCH, (uint32_t)newCCR);
    return;
}

static void pwm_start_timer(uint32_t timPerif){
    //@Breif: Start PWM Generation
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_enable_break_main_output(timPerif); // for advanced timers only
    timer_enable_counter(timPerif);
    return;
}
#endif // PWM_H