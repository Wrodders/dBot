// Motor Driver Firmware

#include "common/common.h"
#include "drivers/serial.h"
#include "inc/imu.h"
#include "inc/coms.h"
#include "inc/services.h"

#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13

// Debug UART 
#define DEBUG_USART		USART1
#define DEBUG_PORT		GPIOB
#define DEBUG_RX		GPIO7 
#define DEBUG_TX		GPIO6



// Bluetooth UART
#define BT_USART		USART2
#define BT_PORT			GPIOA
#define BT_TX			GPIO2
#define BT_RX			GPIO3

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


// ********** GLOBAL STATIC BUFFERS *************************************** // 

#define RB_SIZE 16
// ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};
uint8_t rx2_buf_[RB_SIZE] = {0};
uint8_t tx2_buf_[RB_SIZE] = {0};



// ******* Clock Set Up ****************************************************** //
static void clock_setup(void){
	// Black pill has 25mhz external crystal
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	// Peripheral enables
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_I2C1); // MPU6050
    rcc_periph_clock_enable(RCC_TIM3); // Encoder

	return;
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interrupt	
    systick_interrupt_enable();
	systick_counter_enable();
	return;
}

int main(void){

	// ***** SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick
    
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	
    Serial ser1 = serialInit(USART1, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ, 
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
	serialLinkRB(&ser1);
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
	serialSend(&ser1, (uint8_t *)"Hello World\n", 13);

    Serial serBT = serialInit(USART2, BT_PORT, BT_RX, BT_TX, GPIO_AF7, NVIC_USART2_IRQ,
                            rx2_buf_, ARR_SIZE(rx2_buf_), 
                            tx2_buf_, ARR_SIZE(tx2_buf_));
    serialLinkRB(&serBT);
    serialConfig(&serBT, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    serialSend(&serBT, (uint8_t *)"Hello BT\n", 10);

    MsgFrame rxFrame = {0}; // Buffer  * THIS SHOULD REALLY BE INSIDE A COMS STRUCT/ class 
    MsgFrame txFrame = {0};


	MPU6050_t mpu6050; // = initMPU6050(I2C_PORT, I2C_SCL, I2C_SDA);
    CompFilt compFilt = {.a = 0.8, .dt = 50};
    IMU imu = {0};

	//if(mpu6050.initalized == false){
	//    len = mysprintf((char *)buf, 0, "FAIL: %d :\n", mpu6050.data);
    //    serialSend(&ser1, buf, len);
	//}

    // ***** Create Fixed time Tasks ***** // 
    enum STATE{INIT = 0, CALIB, AUTO, REMOTE} state = INIT;

    TaskHandle blinkTask = createTask(1000); // 1sec = 1 Hz 
    TaskHandle mpu6050Task = createTask(1); // 2ms = 500 Hz 
    TaskHandle imuTask = createTask(50); // 50ms = 200 Hz
    TaskHandle serialTask = createTask(100); // 100ms = 10hz
    imuTask.enable =false;
    mpu6050Task.enable=false;
    uint8_t len = 0;

    uint32_t loopTick;
	for(;;){

        switch (state){
            case INIT:
                break;
            case CALIB:
                break;
            case AUTO:
                loopTick = get_ticks();
                if(CHECK_TASK(blinkTask, loopTick)){
                    gpio_toggle(led.port, led.pin);    
                    blinkTask.lastTick = loopTick;
                }
                if(CHECK_TASK(serialTask, loopTick)){
                    MsgFrame msgframe;
                    // Run Telemetry
                    comsSendMsg(&ser1, ID_IMU, "A:%f", 3.1415973); 
                }
                if(CHECK_TASK(imuTask, loopTick)){
                    filterComplementary(&compFilt, &imu, &mpu6050.accel, &mpu6050.gyro);
                    imuTask.lastTick = loopTick;
                }
                if(CHECK_TASK(mpu6050Task, loopTick)){
                    readMPU6050(&mpu6050);
                    mpu6050Task.lastTick = loopTick;
                }
                break;
            case REMOTE:
                break;
            default:
                break;
        }
	}
}