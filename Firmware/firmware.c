// Motor Driver Firmware

#include "common/common.h"
#include "drivers/serial.h"
#include "sensors/MPU6050.h"


#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13

// Debug UART 
#define DEBUG_USART		USART1
#define DEBUG_PORT		GPIOB
#define DEBUG_RX		GPIO7 // UART1 PB7/8
#define DEBUG_TX		GPIO6

// Bluetooth UART
#define BT_USART		USART6
#define BT_PORT			GPIOA
#define BT_TX			GPIO11
#define BTRX			GPIO12

// MPU6050 IMU @ I2C1 on GPIOB 
#define I2C_PORT		GPIOB 
#define I2C_SDA			GPIO9
#define I2C_SCL			GPIO8

// DRV8833 2-CH PWM DC Motor Controller 
#define PWMA_TIM 		TIM2
#define PWMA_PORT		GPIOA
#define PWMACH1_PIN		GPIO0
#define PWMACH2_PIN		GPIO1
#define PWMB_TIM		TIM2
#define PWMB_PORT		GPIOA
#define PWMBCH3_PIN		GPIO2
#define PWMBCH4_PIN		GPIO3

// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);


	rcc_periph_clock_enable(RCC_USART1);

	rcc_periph_clock_enable(RCC_I2C1);
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


	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	Serial ser1 = serialInit(USART1, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ);
	serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
	serialSend(&ser1, "hello\n", 7);

	MPU6050_t imu = initMPU6050(I2C_PORT, I2C_SCL, I2C_SDA);
	uint8_t l;
	if(imu.initalized == false){
		char ahh[10]; 
		l = mysprintf(ahh, 0, "FAIL: %d :\n", imu.data);
	}

	
	
	uint32_t len = 0;
	uint8_t data[20] = {0};
	uint8_t buf[20] = {0};
	for(;;){
		gpio_toggle(led.port, led.pin);
		readMPU6050(&imu);
		len = mysprintf((char *)buf, 3,"%f:%f:%f:%f\n",get_ticks(), (float )get_ticks()/1000, imu.accel.x, imu.accel.y, imu.accel.z);
		memset(data, 0, 20);
		serialSend(&ser1, buf,len );
		delay(100);
	

	}

	
	
}