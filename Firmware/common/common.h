#ifndef COMMON_H
#define COMMON_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>


#define TICK			1000     // 1ms
#define CPU_FREQ		84000000 // 84Mhz


//  ********* SYS Clock *********************************//
volatile uint32_t _millis = 0;
void sys_tick_handler(void){
	_millis++;
	return;
}

static uint32_t get_ticks(void){
	return _millis;
}

static void delay(uint32_t milleseconds) {
	// @Breif Blocking delay
  uint32_t end_time = get_ticks() + milleseconds;
  while (get_ticks() < end_time) {};
  return;
}

typedef struct  GPIO {
	uint32_t pin;
	uint32_t port;
}GPIO;


#endif // COMMON_h


