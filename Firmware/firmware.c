// Motor Driver Firmware

#include "common/common.h"
#include "drivers/serial.h"
#include "inc/imu.h"
#include "inc/coms.h"
#include "inc/motor.h"

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

#define M_L_TIM 	    TIM2    // Left Motor PWM Timer 
#define M_L_PORT	    GPIOA   // Left Motor PWM Port
#define M_L_CH1		    GPIO0   // Left Motor PWM CH1 Pin
#define M_L_CH2		    GPIO1   // Left Motor PWM CH2 Pin

#define M_R_TIM		    TIM2    // Right Motor PWM Timer 
#define M_R_PORT	    GPIOA   // Right Motor PWM Port
#define M_R_CH1		    GPIO2   // Right Motor PWM CH1 Pin
#define M_R_CH2		    GPIO3   // Right Motor PWM CH2 Pin

#define DRV_EN_PIN      GPIO5   // DRV8833 Enable PIN
#define DRV_EN_PORT     GPIOA   // DRV8833 Enable PORT

#define ENC_CPR         12      
#define ENC_TIM         TIM3
#define ENC_L_PORT      GPIOB
#define ENC_L_A         GPIO4   // TIM3 CH1
#define ENC_L_B         GPIO5   // TIM3 CH2

#define ENC_R_PORT      GPIOB
#define ENC_R_A         GPIO0   // TIM3 CH3
#define ENC_R_B         GPIO1   // TIM3 CH4


// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 64
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

	// ***** HARDWARE SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick
    
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	
    Serial ser1 = serialInit(DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ,
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    // Coms Messages
    MsgFrame rxFrame = {0}; 
    MsgFrame txFrame = {0};

    // IMU
	MPU6050 mpu6050 = mpu6050Init(I2C_PORT, I2C_SCL, I2C_SDA);
    if(mpu6050.initalized == false){
        serialWrite(&ser1, "MPU605 FAIL\n", 13);
    }else{
        serialWrite(&ser1, "MPU6050 SUCCESS\n", 17);
    }

    IMU imu = imuInit(0.5f, 0.01f);


    // Motor 

    Encoder enc = encoderInit(ENC_TIM, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT,ENC_CPR*20);

    enum STATE{
        T_SCHEDULE = 0, // ** Run Round Robin Scheduler  ** //
        T_POST,         // ** PowerOnSelfTest ** //
        T_CALIB,        // ** Calibrate Robot ** //
        T_BLINK,        // ** Debug WatchDog ** //
        T_COMS,         // ** Execute Cmds & Send Messages ** // 
        T_RESET,       // ** Reset System Cleanup ** //
        T_LL_CTRL, // ** DC MOTOR PI Controller @1kHz  ** //
    }state = T_POST;

    // ***** Application Tasks ***** // 
    TaskHandle blinkTask = createTask(500); // 500ms = 2Hz
    TaskHandle comsTask = createTask(100); // 100ms = 10hz
    TaskHandle llCtrlTask = createTask(50); // 50ms = 200Hz
    uint8_t buf[40];
    uint8_t len = 0;
    float filtAccelxX;
    uint32_t loopTick;
	for(;;){
        loopTick = get_ticks();

        if(CHECK_TASK(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;
        }

        if(CHECK_TASK(comsTask, loopTick)){
            
            //len = mysprintf(buf, 4, "%f:%f:%f:%f\n", mpu6050.accel.x,imu.flitAccel.x, mpu6050.gyro.y,imu.filtGyro.y );
            len = mysprintf(buf, 4, "%d\n", encoderRead(&enc));
            serialSend(&ser1, buf, len);
            comsTask.lastTick = loopTick;
        }

        if(CHECK_TASK(llCtrlTask, loopTick)){
            // Get Pitch Angle

            mpu6050Read(&mpu6050);
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // apply digital LPF to raw measurements
            llCtrlTask.lastTick = loopTick;
        }
	}

}