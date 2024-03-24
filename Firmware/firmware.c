// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/robot.h"
#include "inc/calib.h"



// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 64          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};



int main(void){

	// ***** HARDWARE SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick
    
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
	
    Serial ser1 = serialInit(DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, 
                            NVIC_USART1_IRQ, 
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);


    // ***** COMMUNICATIONS *************************** // 
    const Topic pubMap[NUM_PUBS] = {
        {{.pubId = PUB_CMD_RET}, .name = "CMD_RET", .format ="%c:%s"}, // CMD_ID : RET_VAL
        {{.pubId = PUB_ERROR}, .name = "ERROR", .format = "%s"}, // Error Message
        {{.pubId = PUB_INFO}, .name = "INFO", .format = "%s"}, // System INFO
        {{.pubId = PUB_DEBUG}, .name = "DEBUG", .format = "%s"}, // Debug prints 

        // Application Publisher Topics 
        {{.pubId = PUB_IMU}, .name = "IMU", .format = "%0.2f:%0.2f"}, // ROLL:PITCH:YAW
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%0.2f:%0.2f:%0.2f:%0.2f"} // LSPEED:RSPEED:TL:TR
    };

    const Topic cmdMap[NUM_CMDS] = {
        {{.cmdId = CMD_ID }, .name = "IDENT", .format = "" },
        {{.cmdId = CMD_RESET}, .name = "RESET", .format = ""},

        // Application Cmd Topics
        {{.cmdId = CMD_HELLO}, .name = "HELLO", .format = ""}
    };

    Coms coms = comsInit(pubMap, cmdMap);
    comsSendMsg(&coms, &ser1, PUB_INFO, "Hello PC");

    //***** Two Wheel Self Balancing Robot ************* //
    Robot bot = robotInit();
    delay(1000);

    //***** IMU *************************************** //
	MPU6050 mpu6050 = mpu6050Init(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);    
    IMU imu = imuInit(0.5f, 0.01f);


    // ******** SELF TESTS CALIBRATION ************** // 
    float chip_array[CHIRP_BUF_SIZE];
    ChirpTest chirp = chirpInit(chip_array, ARR_SIZE(chip_array));
    chirpComputeLUT(&chirp,MIN_FREQUENCY, MAX_FREQUENCY, CHIRP_PERIOD_S, 
                    CHIRP_W1, CHIRP_W2, CHIRP_M);
    
    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // PUB SUB RPC
    FixedTimeTask speedCtrlTask = createTask(SPEEDCTRL_PERIOD); // Motor PI Loop
    FixedTimeTask balanceTask = createTask(BALANCE_PERIOD); // IMU Process Loop
    FixedTimeTask sweepTask = createTask(50); // init at 1Hz
  




    // ****** Loop Parameters ******* // 
    // Define loop global variables
    float chirpValue = 0; // Value of the sine function
    double time =0;
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();
        // ********* FIXED TIME TASKS ************ // 
        //@Brief: Uses SysTick to execute periodic tasks
        //@Note: Tasks must be non blocking
        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(comsTask, loopTick)){
            comsSendMsg(&coms, &ser1, PUB_ODOM, bot.motorL.angularVel,bot.motorR.angularVel,
                                                bot.motorL.pi.target, bot.motorR.pi.target);
            comsSendMsg(&coms, &ser1, PUB_IMU, imu.pitch, imu.roll);
            comsTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(speedCtrlTask, loopTick)){
            //@Brief: DC Motor Speed Control Process 
            //@Description: Drives the mobile robot according to 
            motorSpeedCtrl(&bot.motorL);
            motorSpeedCtrl(&bot.motorR);
            speedCtrlTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(balanceTask, loopTick)){
            // Get Pitch Angle
            mpu6050Read(&mpu6050);
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // apply digital LPF to raw measurements
            imuRawEuler(&imu.flitAccel, &imu.roll, &imu.pitch);
            balanceTask.lastTick = loopTick;
        }


        // Inside CHECK_PERIOD state code
        if (CHECK_PERIOD(sweepTask, loopTick)) {

            int dir = chirp.rampDir == 1 ? -1 : 1;
            chirpValue = chirpNext(&chirp) * dir * 0.5;
            // Normalize sine value to keep amplitude constant
            
            // Drive robot with sine wave signal
            robotTankDrive(&bot, chirpValue, chirpValue) ;

            // Update last tick
            sweepTask.lastTick = loopTick;
        }

        //sineSweepTest(&bot, loopTick);





	}

}



