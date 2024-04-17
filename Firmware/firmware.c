// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/ddmr.h"
#include "inc/control.h"

// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 128          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};

int main(void){

    enum STATE {INIT, PAUSED, RUN} state = INIT;


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
        {{.pubId = PUB_IMU}, .name = "IMU", .format = "%0.2f:%0.2f:%0.2f:%0.2f"}, // ROLL:PITCH:YAW
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"} // LSPEED:RSPEED:TL:TR
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
    DDMR ddmr = ddmrInit();
    IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (CTRL_PERIOD * MS_TO_S));
    
    Controller cntrl = cntrlInit();
    cntrlAngVel(&cntrl, 0.0f);
    cntrlLinVel(&cntrl, 0.0f);
    cntrlTheta(&cntrl, 0.0f);
    



    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // PUB SUB RPC
    FixedTimeTask ctrlTask = createTask(CTRL_PERIOD); // Motor PI Loop
    FixedTimeTask mCtrlTask = createTask(MCTRL_PERIOD);

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
                if(_fabs(imu.kalPitch) <= BAL_CUTOFF){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "PAUSED => RUN ");
                    motorEnable(&ddmr.motorL);
                    motorEnable(&ddmr.motorR);
                    state = RUN;
                }
                break;
            case RUN:
                if(_fabs(imu.kalPitch) >= BAL_CUTOFF){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "RUN => PAUSED ");
                    motorDisable(&ddmr.motorL);
                    motorDisable(&ddmr.motorR);
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
            comsSendMsg(&coms, &ser1, PUB_ODOM, ddmr.motorL.angularVel, ddmr.motorR.angularVel,
                                                ddmr.motorL.pi.ref, ddmr.linVel,
                                                cntrl.motionCtrl.out,cntrl.balanceCtrl.out);
            comsSendMsg(&coms, &ser1, PUB_IMU,  imu.comp.pitch, imu.comp.roll, imu.kalPitch,imu.kalRoll );
            comsTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(mCtrlTask, loopTick)){
            ddmrOdometry(&ddmr);
            float mSpeed = ddmr.linVel;
            pidRun(&cntrl.motionCtrl, mSpeed);
            float trgtTheta = cntrl.motionCtrl.out;
            cntrlTheta(&cntrl, trgtTheta);
        }
        if(CHECK_PERIOD(ctrlTask, loopTick)){
            //@Brief: DC Motor Speed Control Process 
            //@Description: Drives the mobile robot according to 
            // trgtVel -> Motion Ctrl -> refAngle -> Balance -> refVel -> DDMR --> refAngVel -> Speed Ctrl
            // trgtAngVel -> DDMR -> refAngVel
            imuRunFusion(&imu);
            imuKalFilt(&imu);
            ddmrOdometry(&ddmr);
            pidRun(&cntrl.balanceCtrl, imu.kalPitch);
            motorSetVel(&ddmr.motorL, cntrl.balanceCtrl.out);
            motorSetVel(&ddmr.motorR, cntrl.balanceCtrl.out);
            
            // Motor Speed PI 
            motorSpeedCtrl(&ddmr.motorL);
            motorSpeedCtrl(&ddmr.motorR);
            ctrlTask.lastTick = loopTick;
        }
    }
}