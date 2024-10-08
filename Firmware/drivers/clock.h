
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

#define TICK			1000     // 1ms
#define CPU_FREQ		84000000 // 84Mhz

volatile uint32_t _millis = 0;
void sys_tick_handler(void){
	_millis++;
	return;
}

static uint32_t get_ticks(void){
	return _millis;
}

static void delay(uint32_t milliseconds) {
	// @Brief Blocking delay
  uint32_t end_time = get_ticks() + milliseconds;
  while (get_ticks() < end_time) {};
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interrupt	
    systick_interrupt_enable();
	systick_counter_enable();
}


static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_USART1);


	rcc_periph_clock_enable(RCC_I2C1); // MPU6050
    rcc_periph_clock_enable(RCC_TIM2); // Motor PWM
    rcc_periph_clock_enable(RCC_TIM3); // Encoder L Quaducore Input capture
    rcc_periph_clock_enable(RCC_TIM4); // Encoder R Quaducore Mode
}

#endif // SYSTICK_H