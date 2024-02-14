// Serial Device Driver Test

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>



#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13



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

static GPIO initGPIO(uint32_t pin, uint32_t port, uint32_t mode, uint32_t pupd) {
	GPIO p;
	p.pin = pin;
	p.port = port;

	gpio_mode_setup(p.port, mode, pupd, p.pin);
	return p;
}


// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
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

	GPIO LED = initGPIO(LED_PIN, LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	for(;;){
		gpio_toggle(LED.port, LED_PIN);
		delay(1000);
	}
}