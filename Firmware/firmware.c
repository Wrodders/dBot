// Motor Driver Firmware

#include "common/common.h"

#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13


// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// GPIO
	rcc_periph_clock_enable(RCC_GPIOC);

	return;
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interupt	
    systick_interrupt_enable();
	systick_counter_enable();
	return;
}

// **** GPIO Setup ************************************************************** //
static GPIO initGPIO(uint32_t pin, uint32_t port, uint32_t mode, uint32_t pupd) {
	GPIO p;
	p.pin = pin;
	p.port = port;

	gpio_mode_setup(p.port, mode, pupd, p.pin);
	return p;
}

int main(void){
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick

	GPIO Led = initGPIO(LED_PIN, LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);

	for(;;){
		gpio_toggle(Led.port, Led.pin);
		delay(500);
	}
}