// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/robot.h"



// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 64          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};



int main(void){

	// ***** HARDWARE SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick
    
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	
    Serial ser1 = serialInit(DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, NVIC_USART1_IRQ,
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);


    // ***** COMMUNICATIONS *************************** // 
    const Topic pubMap[NUM_PUBS] = {
        {{.pubId = PUB_CMD_RET}, .name = "CMD_RET", .format ="%c:%s"}, // CMD_ID : RET_VAL
        {{.pubId = PUB_ERROR}, .name = "ERROR", .format = "%s"}, // Error Message
        {{.pubId = PUB_INFO}, .name = "INFO", .format = "%0.3f:%0.3f:%0.3f"}, // System INFO
        {{.pubId = PUB_DEBUG}, .name = "DEBUG", .format = "%s"}, // Debug prints 

        // Application Publisher Topics 
        {{.pubId = PUB_IMU}, .name = "IMU", .format = "%0.2f:%0.2f:%0.2f"}, // ROLL:PITCH:YAW
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%d:%d"} // LSPEED:RSPEED
    };

    const Topic cmdMap[NUM_CMDS] = {
        {{.cmdId = CMD_ID }, .name = "IDENT", .format = "" },
        {{.cmdId = CMD_RESET}, .name = "RESET", .format = ""},

        // Application Cmd Topics
        {{.cmdId = CMD_HELLO}, .name = "HELLO", .format = ""}
    };

    Coms coms = comsInit(pubMap, cmdMap);

    comsSendMsg(&coms, &ser1, PUB_INFO, "Hello PC");

    // Coms Messages
    MsgFrame rxFrame = {0}; 
    MsgFrame txFrame = {0};

    // IMU
	MPU6050 mpu6050 = mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);    
    IMU imu = imuInit(0.5f, 0.01f);
    // Robot ROBOT
    Robot robot = robotInit();
    delay(1000);
    
    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // 500ms = 2Hz
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // 100ms = 10hz
    FixedTimeTask speedCtrlTask = createTask(SPEEDCTRL_PERIOD); // 50ms = 200Hz

    // ****** Loop Parameters ******* // 
    uint8_t buf[40]; // tx coms buffer
    uint8_t len = 0; // msg len
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();

        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;
        }

        if(CHECK_PERIOD(comsTask, loopTick)){
            char* pubFmt = comsGetPubFmt(&coms, PUB_IMU);
            comsSendMsg(&coms, &ser1, PUB_INFO, imu.roll, imu.pitch, imu.yaw);
            //serialSend(&ser1, (uint8_t* )pubFmt, uClen(pubFmt));
            comsTask.lastTick = loopTick;
        }

        if(CHECK_PERIOD(speedCtrlTask, loopTick)){
            // Get Pitch Angle
            mpu6050Read(&mpu6050);
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // apply digital LPF to raw measurements
            imuRawEuler(&imu.flitAccel, &imu.roll, &imu.pitch);

            robotSpeedCtrl(&robot); // run pid


            speedCtrlTask.lastTick = loopTick;

        }
	}
}