#ifndef COMMON_H
#define COMMON_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>

#include "ringbuffer.h"
#include "utils.h"


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


#define CHECK_TASK(TASK, LOOPTICK) 	((LOOPTICK - TASK.lastTick >= TASK.rate) && (TASK.enable))



typedef struct TaskHandle{
    uint32_t lastTick;
    uint32_t rate;	
	bool enable;
}TaskHandle;

static TaskHandle createTask(uint32_t rate){
	TaskHandle task;
	task.lastTick = get_ticks();
	task.rate = rate;
	task.enable = true;
	return task;
}

static void disableTask(TaskHandle *t){
	t->enable = false;
	return;
}

static void enableTask(TaskHandle *t){
	t->enable = true;
	return;
}

#endif // COMMON_h


