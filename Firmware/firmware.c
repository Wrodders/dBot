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

// ********* SUPER LOOP **************** // 
int main(void){

    enum STATE {INIT = 0, PARK, RUN} state = INIT;
    enum MODE {MCNTRL = 0, PIDCNTRL,LQRCNTRL} mode = PIDCNTRL;

    // ***************************** COMMUNICATIONS ******************************************************** // 
    const Topic pubMap[NUM_PUBS] = {
        {{.pubId = PUB_CMD_RET}, .name = "CMD_RET", .format ="%s:%s", .nArgs=2}, // CMD_ID : RET_VAL
        {{.pubId = PUB_ERROR}, .name = "ERROR", .format = "%s", .nArgs=1}, // Error Message
        {{.pubId = PUB_INFO}, .name = "INFO", .format = "%s", .nArgs=1}, // System INFO
        {{.pubId = PUB_DEBUG}, .name = "DEBUG", .format = "%s", .nArgs=1}, // Debug prints 

        // Application Publisher Topics 
        {{.pubId = PUB_IMU}, .name = "IMU", .format = "%0.3f:%0.3f", .nArgs=2}, // ROLL:PITCH:YAW
        {{.pubId = PUB_ODOM}, .name = "ODOM", .format = "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f", .nArgs=4} // LSPEED:RSPEED:TL:TR:LIN:BAL
    };
    const Topic cmdMap[NUM_CMDS] = {
        {{.cmdId = CMD_ID }, .name = "IDENT", .format = "", .nArgs=0},
        {{.cmdId = CMD_RESET}, .name = "RESET", .format = "", .nArgs=0},

        // Application Cmd Topics
        {{.cmdId = CMD_MODE}, .name = "MODE", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_WT}, .name = "WTRGT", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_WP}, .name = "WP", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_WI}, .name = "WI", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_BP}, .name = "BP", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_BI}, .name = "BI", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_BD}, .name = "BD", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_MT}, .name = "MTRGT", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_MP}, .name = "MP", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_MI}, .name = "MI", .format = "%f", .nArgs=1},
        {{.cmdId = CMD_MI}, .name = "MD", .format = "%f", .nArgs=1}
    };

	// ***************************** HARDWARE SETUP ******************************************************** //
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
    IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (IMU_FUSION_PERIOD * MS_TO_S));
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
    PID balanceCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (BAL_CNTRL_PERIOD * MS_TO_S));
    pidSetRef(&balanceCtrl, BAL_OFFSET);
    DDMR ddmr = {.dt = VEL_CNTRL_PERIOD * MS_TO_S};
    PID motionCtrl = pidInit(-12, 12, VEL_P, VEL_I, VEL_D, (VEL_CNTRL_PERIOD * MS_TO_S));
    pidSetRef(&motionCtrl, 0.0f);


    // ***** Application Tasks **************************************************************************** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD);              // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD);                // PUB SUB RPC
    FixedTimeTask imuFusionTask = createTask(IMU_FUSION_PERIOD);     // Sensor Fusion 

    FixedTimeTask wspeedCntlTask = createTask(WSPEED_CNTRL_PERIOD);  //wheel Speed Control (PI)
    FixedTimeTask balanceCntrlTsk = createTask(BAL_CNTRL_PERIOD);    // Balance Theta Control
    FixedTimeTask velCntrlTask = createTask(VEL_CNTRL_PERIOD);        // State Control Loop
    // ****** Loop Parameters **************************************************************************** // 
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();
        switch (state){
            case INIT:
                  if(get_ticks() >= 3000){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "INIT => PARK ");
                    systick_clear();
                    state = PARK;
                  }
                  break;
            case PARK:
                if(_fabs(imu.kal.pitch) <= BAL_CUTOFF){
                    comsSendMsg(&coms, &ser1, PUB_INFO, "PAUSED => RUN ");
                    motorEnable(&motorL);
                     motorEnable(&motorR);
                    state = RUN;
                }
                break;
            case RUN:
                if(_fabs(imu.kal.pitch) >= BAL_CUTOFF){  
                    comsSendMsg(&coms, &ser1, PUB_INFO, "RUN => PARK "); // Safety Cut Off
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    state = PARK;
                }
                switch(mode){
                    case MCNTRL:
                        // Controls Motor Target Speed (RPM)
                        break;
                    case PIDCNTRL:
                        // Controls Robot X Velocity from Odometry 
                        // Adjusts Target balance angle
                        // Controls Wheel Voltage from balance angle
                        if(CHECK_PERIOD(balanceCntrlTsk, loopTick)){
                            // Balance Control Inverted Pendulum PID
                            pidRun(&balanceCtrl, imu.kal.pitch); 
                            motorSetTrgtVel(&motorL, balanceCtrl.out); // Run Both Motors Same Speed
                            motorSetTrgtVel(&motorR, balanceCtrl.out);
                            balanceCntrlTsk.lastTick = loopTick;
                        }
                        if(CHECK_PERIOD(velCntrlTask, loopTick)){
                            ddmrOdometry(&ddmr, &motorL, &motorR);

                            // Velocity Control Differential Drive Robot
                            //pidRun(&motionCtrl, ddmr.linVel);
                            //pidSetRef(&balanceCtrl, BAL_OFFSET - motionCtrl.out);
                            velCntrlTask.lastTick = loopTick;

                        }
                        break;
                    case LQRCNTRL:
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;  
        };

        // ********* FIXED TIME TASKS ******************************************************************* // 

        if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
            // Motor Speed Control PI 
            motorCalSpeed(&motorL);
            motorCalSpeed(&motorR);
            pidRun(&motorL.pi, motorL.angularVel);
            pidRun(&motorR.pi, motorR.angularVel);
            motorSetVoltage(&motorL, motorL.pi.out);
            motorSetVoltage(&motorR, motorR.pi.out);
            wspeedCntlTask.lastTick = loopTick;
        }

        if(CHECK_PERIOD(imuFusionTask, loopTick)){
            mpu6050Read(&mpu6050); // Read Sensor
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // Apply Low pass filter
            imuCompFilt(&imu);  // Estimate Angle with  Complementary Filter
            imuKalUpdate(&imu); // Estimate Angle with Kalman Observer
            imuFusionTask.lastTick = loopTick;
        }


        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;  
        }
        if(CHECK_PERIOD(comsTask, loopTick)){
            // TX
            comsSendMsg(&coms, &ser1, PUB_IMU,imu.kal.pitch, imu.kal.roll);
            comsSendMsg(&coms, &ser1, PUB_ODOM,motorL.angularVel, motorR.angularVel, motorL.pi.ref, motorR.pi.ref, ddmr.linX, motionCtrl.out);
            // RX
            if(comsGrabMsg(&coms, &ser1)){
                switch(coms.rxFrame.buf[0]){
                    case 'a': // GET CMDS
                        char valBuf[7]; 
                        switch(coms.rxFrame.id){
                            case CMD_ID:
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET, "a", "STM");
                                break;
                            case CMD_RESET:
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET, "b", "RESET");
                                break;
                            case CMD_MODE:
                                if(mode == MCNTRL){ comsSendMsg(&coms, &ser1, PUB_CMD_RET, "c", "MCNTRL");}
                                else if(mode == PIDCNTRL){ comsSendMsg(&coms, &ser1, PUB_CMD_RET, "c", "PIDCNTRL");}
                                else if(mode == LQRCNTRL){ comsSendMsg(&coms, &ser1, PUB_CMD_RET, "c", "LQRCNTRL");}
                                break;
                            case CMD_WT:
                                snprintf(valBuf, 7, "%0.3f", motorL.pi.ref);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"d",valBuf);
                                break;
                            case CMD_WP:
                                snprintf(valBuf, 7, "%0.3f", motorL.pi.kp);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"e",valBuf);
                                break;
                            case CMD_WI:
                                snprintf(valBuf, 7, "%0.3f", motorL.pi.ki);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"f",valBuf);
                                break;
                            case CMD_BP:
                                snprintf(valBuf, 7, "%0.3f", balanceCtrl.kp);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"g",valBuf);
                                break;
                            case CMD_BI:
                                snprintf(valBuf, 7, "%0.3f", balanceCtrl.ki);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"h",valBuf);
                                break;
                            case CMD_BD:
                                snprintf(valBuf, 7, "%0.3f", balanceCtrl.kd);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"i",valBuf);
                                break;
                            case CMD_MT:
                                snprintf(valBuf, 7, "%0.3f", motionCtrl.ref);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"j",valBuf);
                                break;
                            case CMD_MP:
                                snprintf(valBuf, 7, "%0.3f", motionCtrl.kp);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"k",valBuf);
                                break;
                            case CMD_MI:
                                snprintf(valBuf, 7, "%0.3f",motionCtrl.ki);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"l",valBuf);
                                break;
                            case CMD_MD:
                                snprintf(valBuf, 7, "%0.3f",motionCtrl.kd);
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET,"i",valBuf);
                                break;
                            default:
                                comsSendMsg(&coms, &ser1, PUB_ERROR, "INVALID RREF");
                                break;
                        }
                        break;
                    case 'b': // SET CMDS
                        float val;
                        val = atof((char*)&coms.rxFrame.buf[1]); // read in paramater value 
                        switch(coms.rxFrame.id){
                            case CMD_MODE:
                                comsSendMsg(&coms, &ser1, PUB_CMD_RET, "c", &coms.rxFrame.buf[1]);
                                if(val == MCNTRL) { // Reset Top Level Controller Target
                                    pidSetRef(&motorL.pi, 0.0f);
                                    pidSetRef(&motorR.pi, 0.0f);
                                    mode = MCNTRL; 
                                }else if (val == PIDCNTRL){
                                    pidSetRef(&motionCtrl, 0.0f);
                                    pidSetRef(&motionCtrl, 0.0f);
                                    mode = PIDCNTRL;
                                }
                                break;
                            case CMD_WT:
                                pidSetRef(&motorL.pi, val);
                                pidSetRef(&motorR.pi, val);
                                break;
                            case CMD_WP:
                                pidSetKp(&motorL.pi,val); 
                                pidSetKp(&motorR.pi,val); 
                                break;
                            case CMD_WI:
                                pidSetKi(&motorL.pi,val); 
                                pidSetKi(&motorR.pi,val); 
                                break;
                            case CMD_BP:
                                pidSetKp(&balanceCtrl,val); 
                                break;
                            case CMD_BI:
                                pidSetKi(&balanceCtrl,val); 
                                break;
                            case CMD_BD:
                                pidSetKd(&balanceCtrl,val); 
                                break;
                            case CMD_MT:
                                pidSetRef(&motionCtrl,val); 
                                break;
                            case CMD_MP:
                                pidSetKp(&motionCtrl,val); 
                                break;
                            case CMD_MI:
                                pidSetKi(&motionCtrl, val);
                                break;
                            case CMD_MD:
                                pidSetKd(&motionCtrl, val);
                                break;
                            default:
                                comsSendMsg(&coms, &ser1, PUB_ERROR, "INVALID WREG");
                                break;
                        }
                        break;
                    default:
                        comsSendMsg(&coms, &ser1, PUB_ERROR, "INVALID CMDTYPE");
                        break;
               }
            }
            comsTask.lastTick = loopTick;
        }

        
    }
}