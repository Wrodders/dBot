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
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// GPIO
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);

	// USART
	rcc_periph_clock_enable(RCC_USART1);

	// Timer
	rcc_periph_clock_enable(RCC_TIM2); 

	// I2C
	rcc_periph_clock_enable(RCC_I2C1);

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

	Serial ser1 = serial_init(USART1, GPIOB, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ);
	serial_config(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
	serial_begin(&ser1); 

	// Initialize Timer PWM
    pwm_config_timer(TIM2,84,25000); // set freq to period to 25000 ARR reg
	pwm_init_output_channel(TIM2, TIM_OC1,GPIOA, GPIO0, GPIO_AF1); 
	pwm_start_timer(TIM2);
	pwm_set_dutyCycle(TIM2, TIM_OC1, 0.5f); // set to 50%
	
	// Initialize I2C Sensor	
	serial_write(&ser1, (uint8_t *)"IMU INIT BEGIN\n", 16);
	MPU6050_t imu = initMPU6050(SDA_PIN, SCL_PIN, GPIOB);
	if(!imu.initalized){
		serial_write(&ser1, (uint8_t *)"IMU INIT FAIL\n", 15);
	}
	MadgwickFilter mf = initMadgwick(500, 0.1f);


	TaskHandle blinkTask = createTask(500);
	TaskHandle uartTask = createTask(100);
	TaskHandle imuTask = createTask(2);
	//disableTas(&imuTask)
	
	for(;;){
		uint32_t loopTick = get_ticks();
		if(CHECK_TASK(blinkTask, loopTick)){
			gpio_toggle(Led.port, Led.pin);
			blinkTask.lastTick = loopTick;
		}else if(CHECK_TASK(uartTask, loopTick)){
			char buffer[60];
			int datalen = mysprintf(buffer, 4,"%f:%f\n", imu.roll, imu.pitch );
			serial_write(&ser1, (uint8_t *)buffer, datalen);
			uartTask.lastTick = loopTick;
		}else if(CHECK_TASK(imuTask, loopTick)){
			readMPU6050(&imu);
    		computeAngles(&imu.gyro, &imu.accel, &imu.roll, &imu.pitch);
			MadgwickAHRSupdateIMU(&mf, imu.gyro.x, imu.gyro.y, imu.gyro.z, imu.accel.x, imu.accel.y, imu.accel.z);
			imuTask.lastTick = loopTick;
		}

		delay(500);
	}
}