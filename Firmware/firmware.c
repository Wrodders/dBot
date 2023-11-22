// Motor Driver Firmware

#include "common/common.h"
#include "drivers/serial.h"
#include "drivers/pwm.h"
#include "inc/MPU6050.h"

#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13

#define DEBUG_RX		GPIO7
#define DEBUG_TX		GPIO6

#define SDA_PIN			GPIO8
#define SCL_PIN			GPIO9

// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// GPIO
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOD);

	// USART
	rcc_periph_clock_enable(RCC_USART1);


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

	Serial ser1 = serialInit(USART1, GPIOA, GPIO10, GPIO9, GPIO_AF7, NVIC_USART1_IRQ);
	serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);

	TaskHandle uartTask = createTask(100);

	GPIO LED = initGPIO(GPIO5, GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	float pi = 3.141;
	for(;;){
		uint32_t loopTick = get_ticks();
		gpio_toggle(GPIOA, GPIO5);
		delay(1000);
		
		if(CHECK_TASK(uartTask, loopTick)){
			char buffer[50];
			pi =pi+0.001;

		
			uartTask.lastTick = loopTick;
		}


	}
}