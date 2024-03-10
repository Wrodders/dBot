
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


#endif // SYSTICK_H