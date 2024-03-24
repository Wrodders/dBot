// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/ddmr.h"
#include "inc/twsb.h"

// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 128          // ACCESS THROUGH RING BUFFER 
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
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%0.2f:%0.2f:%0.2f:%0.2f:%0.2f"} // LSPEED:RSPEED:TL:TR
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
    DDMR dBot = ddmrInit();
    TWSB bBot = twsbInit();
    twsbSetRef(&bBot, 0.0f);
    
    delay(1000);   
    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // PUB SUB RPC
    FixedTimeTask speedCtrlTask = createTask(SPEEDCTRL_PERIOD); // Motor PI Loop
    FixedTimeTask balanceTask = createTask(BALANCE_PERIOD); // IMU Process Loop
    // ****** Loop Parameters ******* // 
    // Define loop global variables
    int rampDir = 1;
    float trgVel = 1;
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
            
            switch(rampDir){

                case 1:
                    if(trgVel >= VEL_MAX){rampDir = -1;}
                    else{trgVel += 0.2f;}
                    break;
                

                case -1:
                    if(trgVel <= -VEL_MAX){rampDir = 1;}
                    else{trgVel -= 0.2f;}
                    break;

            };

            ddmrDrive(&dBot,trgVel, 10.0f);





            comsSendMsg(&coms, &ser1, PUB_ODOM, dBot.motorL.angularVel,dBot.motorR.angularVel,
                                                dBot.motorL.pi.target, dBot.motorR.pi.target,
                                                bBot.balancer.out);
            comsSendMsg(&coms, &ser1, PUB_IMU,bBot.imu.pitch, bBot.imu.roll);
            comsTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(speedCtrlTask, loopTick)){
            //@Brief: DC Motor Speed Control Process 
            //@Description: Drives the mobile robot according to 
            motorSpeedCtrl(&dBot.motorL);
            motorSpeedCtrl(&dBot.motorR);
            speedCtrlTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(balanceTask, loopTick)){
            twsbBalancer(&bBot);
            balanceTask.lastTick = loopTick;
        }
    }
}



