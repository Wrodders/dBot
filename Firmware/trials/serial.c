// Serial Device Driver Test

#include "../common/common.h"
#include "../drivers/serial.h"


#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13

#define DEBUG_TX		GPIO9
#define DEBUG_RX		GPIO10
#define DEBUG_PORT		GPIOA



// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
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

	Serial ser1 = serialInit(USART1, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ);
	serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);

	GPIO LED = initGPIO(LED_PIN, LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);

	TaskHandle txTask = createTask(100);
	TaskHandle rxTask = createTask(100);
	TaskHandle blinkTask = createTask(500);

	uint32_t len = 0;
	uint8_t data[20] = {0};
	uint32_t loopTick;
	for(;;){
		gpio_toggle(LED.port, LED_PIN);
		len = serialGrab(&ser1, data, 10);
		serialSend(&ser1, data, len);
		delay(1000);
	}
}