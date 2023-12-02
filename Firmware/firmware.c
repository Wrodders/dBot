// Motor Driver Firmware

#include "common/common.h"
#include "drivers/serial.h"
#include "drivers/pwm.h"
#include "inc/MPU6050.h"

#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13

#define DEBUG_TX		GPIO6
#define DEBUG_RX		GPIO7
#define DEBUG_PORT		GPIOB

#define SDA_PIN			GPIO8
#define SCL_PIN			GPIO9

// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_USART1);
	return;
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interupt	
    systick_interrupt_enable();
	systick_counter_enable();
	return;
}


int main(void){

	// ***** SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick

	for(;;){
		
	}

	
	
}