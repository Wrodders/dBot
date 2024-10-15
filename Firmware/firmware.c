// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/MPU6050.h"
#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/pid.h"
#include "inc/motor.h"

#include "inc/ddmr.h"


// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 128          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};

int main(void){

    enum STATE {INIT, PAUSED, RUN} state = INIT;


     // ***** COMMUNICATIONS *************************** // 
    const Topic pubMap[NUM_PUBS] = {
        {{.pubId = PUB_CMD_RET}, .name = "CMD_RET", .format ="%c:%s"}, // CMD_ID : RET_VAL
        {{.pubId = PUB_ERROR}, .name = "ERROR", .format = "%s"}, // Error Message
        {{.pubId = PUB_INFO}, .name = "INFO", .format = "%s"}, // System INFO
        {{.pubId = PUB_DEBUG}, .name = "DEBUG", .format = "%s"}, // Debug prints 

        // Application Publisher Topics 
        {{.pubId = PUB_IMU}, .name = "IMU", .format = "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"}, // ROLL:PITCH:YAW
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"} // LSPEED:RSPEED:TL:TR
    };
    const Topic cmdMap[NUM_CMDS] = {
        {{.cmdId = CMD_ID }, .name = "IDENT", .format = "" },
        {{.cmdId = CMD_RESET}, .name = "RESET", .format = ""},

        // Application Cmd Topics
        {{.cmdId = CMD_HELLO}, .name = "HELLO", .format = ""}
    };


	// ***** HARDWARE SETUP ********** //
	clock_setup(); // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup(); // 1ms Tick
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
    // COMMUNICATIONS
    Serial ser1 = serialInit(DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, 
                            NVIC_USART1_IRQ, 
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    Coms coms = comsInit(pubMap, cmdMap);
    comsSendMsg(&coms, &ser1, PUB_DEBUG, "COMS");
    // IMU 
    i2cInit(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);
    comsSendMsg(&coms, &ser1, PUB_DEBUG, "I2C");
    MPU6050 mpu6050 = mpu6050Init(IMU_PERIF);
    IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (CTRL_PERIOD * MS_TO_S));
    comsSendMsg(&coms, &ser1, PUB_DEBUG, "IMU");
    // MOTORS 
    Encoder encL = encoderInit(ENC_L_TIM, UINT16_MAX, ENC_L_A, ENC_L_PORT, ENC_L_B, ENC_L_PORT, ENC_L_AF);
    Encoder encR = encoderInit(ENC_R_TIM, UINT16_MAX, ENC_R_A, ENC_R_PORT, ENC_R_B, ENC_R_PORT, ENC_R_AF);
    Motor motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    Motor motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    motorDisable(&motorL); // Disable DRV8833
    motorDisable(&motorR); // Disable DRV8833
    motorConfig(&motorL, &encL, VBAT_MAX, 1.3f, true, SPEED_ALPHA, RPS_MAX);
    motorConfig(&motorR, &encR, VBAT_MAX, 1.3f, false,  SPEED_ALPHA, RPS_MAX);
    // BALANCE CONTROLLER
    PID balanceCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (CTRL_PERIOD * MS_TO_S));
    pidSetRef(&balanceCtrl, BAL_OFFSET);
    DDMR ddmr = {0};
    PID motionCtrl = pidInit(-VEL_MAX, VEL_MAX, VEL_P, VEL_I, VEL_D, (CTRL_PERIOD * MS_TO_S));
    pidSetRef(&motionCtrl, 0.0f);


    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // PUB SUB RPC
    FixedTimeTask ctrlTask = createTask(CTRL_PERIOD); // State Control Loop
    // ****** Loop Parameters ******* // 
    // Define loop global variables
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();
        switch (state){
            case INIT:
                  if(get_ticks() >= 3000){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "INIT => PAUSED ");
                    systick_clear();
                    state = PAUSED;
                  }
                  break;
            case PAUSED:
                if(_fabs(imu.kal.pitch) <= BAL_CUTOFF){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "PAUSED => RUN ");
                    motorEnable(&motorL);
                    motorEnable(&motorR);
                    state = RUN;
                }
                break;
            case RUN:
                if(_fabs(imu.kal.pitch) >= BAL_CUTOFF){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "RUN => PAUSED ");
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    state = PAUSED;
                }
                break;
            default:
                break;  
        };

        // ********* FIXED TIME TASKS ************ // 
        //@Brief: Uses SysTick to execute periodic tasks
        //@Note: Tasks must be non blocking
        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(comsTask, loopTick)){
            comsSendMsg(&coms, &ser1, PUB_IMU,  imu.raw.pitch, imu.comp.pitch,imu.kal.pitch, imu.raw.roll, imu.comp.roll ,imu.kal.roll );
            comsTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(ctrlTask, loopTick)){
            //@Brief: DC Motor Speed Control Process 
            //@Description: Drives the mobile robot according to 
            // refAngle -> Balance -> refVel -> DDMR --> refAngVel -> Speed Ctrl
            // Balance Control PID
            mpu6050Read(&mpu6050); // Read Sensor
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // Apply Low pass filter
            imuCompFilt(&imu);  // Estimate Angle with  Complementary Filter
            imuKalUpdate(&imu); // Estimate Angle with Kalman Observer
            ddmrOdometry(&ddmr, &motorL, &motorR);
            pidRun(&motionCtrl, ddmr.linVel);
            pidSetRef(&balanceCtrl, BAL_OFFSET - motionCtrl.out);
            pidRun(&balanceCtrl, imu.kal.pitch); 
            motorSetTrgtVel(&motorL, balanceCtrl.out); // Run Both Motors Same Speed
            motorSetTrgtVel(&motorR, balanceCtrl.out);
            // Motor Speed Control PI 
            motorSpeedCtrl(&motorL);
            motorSpeedCtrl(&motorR);
            ctrlTask.lastTick = loopTick;
        }
    }
}