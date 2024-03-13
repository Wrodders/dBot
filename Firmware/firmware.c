// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"
#include "drivers/systick.h"
#include "drivers/serial.h"
//#include "inc/imu.h"
#include "inc/coms.h"
#include "inc/robot.h"

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
    rcc_periph_clock_enable(RCC_TIM2); // Motor PWM
    rcc_periph_clock_enable(RCC_TIM3); // Encoder L Quducore Input capture
    rcc_periph_clock_enable(RCC_TIM1); // Encoder R Quaducore Mode
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interrupt	
    systick_interrupt_enable();
	systick_counter_enable();
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

    serialSend(&ser1, (uint8_t *)"Hello PC\n", 10);
    // Coms Messages
    MsgFrame rxFrame = {0}; 
    MsgFrame txFrame = {0};

    // IMU
	/*MPU6050 mpu6050 = mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);
    if(mpu6050.initalized == false){
        serialWrite(&ser1, (uint8_t *)"MPU605 FAIL\n", 13);
    }else{
        serialWrite(&ser1, (uint8_t *)"MPU6050 SUCCESS\n", 17);
    }
    */
    //IMU imu = imuInit(0.5f, 0.01f);


    // DDMR ROBOT
    DDMR robot = ddmrInit();
    delay(1000);
    //ddmrTankDrive(&robot,0.5,0.5);
    
    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // 500ms = 2Hz
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // 100ms = 10hz
    FixedTimeTask llCtrlTask = createTask(SPEEDCTRL_PERIOD); // 50ms = 200Hz
    uint8_t buf[40]; // tx coms buffer
    uint8_t len = 0; // msg len
    uint16_t mCount = 0;
    bool dir = 1;
    float mSpeed = 0; // rps
    uint16_t dCount = 0;
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();

        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;
        }

        if(CHECK_PERIOD(comsTask, loopTick)){
            
            uint8_t DATA_IDX = 3;                                                        
            MsgFrame MSG = {0};                                                                
            int SIZE = mysprintf((char *)&MSG.buf[DATA_IDX], 2, "%u:%f:%f:%f",encoderRead(&robot.encL), robot.motorL.angularSpeed, robot.motorR.angularSpeed, robot.pidL.out);   
            if(SIZE <= (MAX_MSG_DATA_SIZE - 1)){                                         
                uint8_t IDX = 0;                                                             
                MSG.buf[IDX++] = SOF_BYTE;                                                   
                MSG.buf[IDX++] = SIZE;                                                       
                MSG.buf[IDX++] = PUB_ODOM;                                                         
                IDX += SIZE;                                                                 
                MSG.buf[IDX++] = EOF_BYTE;                                                   
                serialSend(&ser1, MSG.buf, IDX);                                               
            }

            comsTask.lastTick = loopTick;
        }

        if(CHECK_PERIOD(llCtrlTask, loopTick)){
            // Get Pitch Angle
            //mpu6050Read(&mpu6050);
            //imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // apply digital LPF to raw measurements

            ddmrSpeedCtrl(&robot); // run pid


            llCtrlTask.lastTick = loopTick;

        }
	}
}