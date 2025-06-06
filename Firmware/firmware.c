// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/MPU6050.h"

#include "inc/imu.h"
#include "inc/pid.h"
#include "inc/motor.h"
#include "inc/ddmr.h"
#include "inc/lqr.h"
#include "pubrpc/serComs.h" // Serial Communications to PC

// ********** GLOBAL STATIC BUFFERS ***************************************// ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[512] = {0};
uint8_t tx1_buf_[2048] = {0};
float uuid = 3.14159;
// ********** CASCADED PID FUNCTIONS ***************************************//
//@Brief: Motor Speed Control Loop
static void runMotorSpeedLoop(struct Motor *m){
    m->speedCtrl.out = pidRun(&m->speedCtrl, m->wheelRPS);
    motorSetVoltage(m, m->speedCtrl.out);
}
//@Brief: Balance Angle Control Loop
static void runBalanceLoop(struct PID *balanceAngleCtrl, struct PID *steerCtrl, struct IMU *imu, 
                            struct Motor *motorL, struct Motor *motorR){
    // error = (Reference - Calibrated Offset) - Estimated Pitch
    balanceAngleCtrl->out = pidRun(balanceAngleCtrl, imu->kal.pitch); // Unicycle Wheel Speed output
    motorSetTrgtSpeed(motorL, (balanceAngleCtrl->out + steerCtrl->out)); // Apply Steering Correction
    motorSetTrgtSpeed(motorR, (balanceAngleCtrl->out - steerCtrl->out)); // Apply Steering Correction
}
//@Brief: Differential Drive Velocity Control Loop
static void runVelocityLoop(struct PID *velCtrl,struct PID *steerCtrl, struct PID *balanceAngleCtrl, 
                            struct DiffDriveModel *ddmr){          
    velCtrl->out = pidRun(velCtrl, ddmr->v_b); // Reference - odometry velocity 
    steerCtrl->out = pidRun(steerCtrl,ddmr->w_b); // Reference - odometry angular velocity
    pidSetRef(balanceAngleCtrl, -velCtrl->out); // Sets Balance Reference point to reach desired velocity
}
// ********** LQR FUNCTIONS ***************************************//
static void runLQR(struct LQR *lqr, struct Motor *motorL, struct Motor *motorR){
    float xpos = *lqr->state.x[0];
    float linVel = *lqr->state.x[1];
    float pitch = *lqr->state.x[2] * DEG_TO_RAD;
    float pitchRate = *lqr->state.x[3] * DEG_TO_RAD;
    
    lqr->u = -lqr->K[0] * xpos - lqr->K[1] * linVel - lqr->K[2] * pitch - lqr->K[3] * pitchRate;

    motorSetVoltage(motorL, lqr->u);
    motorSetVoltage(motorR, lqr->u);
}
// ********* SUPER LOOP **************** // 
int main(void){
    float nextMode = 0;
	// ***************************** HARDWARE SETUP ******************************************************** //
	systemClockSetup();      // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup();    // 1ms Tick 
	struct GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
    struct GPIO drvMode = initGPIO(DRV_MODE_PIN, DRV_MODE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN); // Set DRV8876 Mode
    gpio_clear(drvMode.port, drvMode.pin); // Gnd == PWM Mode

    // Serial 
    struct Serial ser1 = serialInit( DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, 
                            NVIC_USART1_IRQ, 
                            rx1_buf_, ARRAY_SIZE(rx1_buf_), 
                            tx1_buf_, ARRAY_SIZE(tx1_buf_));
    serialConfig(&ser1, 230400, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);// Baud > 9*(PacketByteSize*rate)
    // IMU 
    i2cInit(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);
    struct MPU6050 mpu6050 = mpu6050Init(IMU_PERIF);
    struct IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (IMU_PERIOD * MS_TO_S));
    imuLinkSensor(&imu, &mpu6050);
    // MOTORS 
    struct Encoder encL = encoderInit(ENC_L_TIM, UINT16_MAX, ENC_L_B, ENC_L_PORT, ENC_L_A, ENC_L_PORT, ENC_L_AF, true);
    struct Encoder encR = encoderInit(ENC_R_TIM, UINT16_MAX, ENC_R_B, ENC_R_PORT, ENC_R_A, ENC_R_PORT, ENC_R_AF, false);
    struct Motor motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    struct Motor motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA, TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    motorDisable(&motorL); // Ensure Driver is Disabled Before Configuration
    motorDisable(&motorR); //
    motorConfig(&motorL, &encL, VSYS, MOTOR_DEADBAND,VMOTOR_MAX, true, SPEED_ALPHA, RPS_MAX);
    motorConfig(&motorR, &encR, VSYS, MOTOR_DEADBAND,VMOTOR_MAX, false,  SPEED_ALPHA, RPS_MAX );

    // BALANCE CONTROLLER
    struct PID balanceAngleCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (BAL_CNTRL_PERIOD * MS_TO_S));
    pidSetRef(&balanceAngleCtrl, -7.5f);
    // DIFFERENTIAL DRIVE MODEL (LinVel, AngVel)
    struct DiffDriveModel ddmr = ddmrInit(WHEEL_RADIUS, WHEEL_BASE, VEL_ALPHA, STEER_ALPHA, ODOM_PERIOD * MS_TO_S);
    struct PID velCtrl = pidInit(-3, 3, VEL_P, VEL_I, VEL_D, (VEL_CNTRL_PERIOD * MS_TO_S));   
    struct PID steerCtrl = pidInit(-5,5, STEER_KP, STEER_KI, STEER_KD, VEL_CNTRL_PERIOD * MS_TO_S);
    // LQR Controller
    struct LQR lqr ={.K[0] = LQR_K1, .K[1] = LQR_K2, .K[2] = LQR_K3, .K[3] = LQR_K4, 
        .state.x = {&ddmr.posX, &ddmr.v_b, &imu.kal.pitch, &imu.kal.pitchRate}};

    // ***************************** COMMUNICATIONS ******************************************************** // 
    struct Coms coms = comsInit();
    // -------------------- TOPIC PUBLISHERS ---------------------- //
    comsRegisterPub(&coms, PUB_CMD_RET, SERIALIZED_CMDRET_FTM);
    comsRegisterPub(&coms, PUB_ERROR,   SERIALIZED_ERROR_FMT);
    comsRegisterPub(&coms, PUB_INFO,    SERIALIZED_INFO_FMT);
    comsRegisterPub(&coms, PUB_DEBUG,   SERIALIZED_DEBUG_FMT);
    comsRegisterPub(&coms, PUB_TELEM,   SERIALIZED_TELEM_FMT);
    // --------------------PARAMETER REGISTERS ---------------------- //
    comsRegisterParam(&coms, P_ID,      "%f", &uuid);
    comsRegisterParam(&coms, P_MODE,    "%f", &nextMode);
    // - LEFT MOTOR
    comsRegisterParam(&coms, P_LTRGT,   "%f", &motorL.speedCtrl.ref);
    comsRegisterParam(&coms, P_LKP,     "%f", &motorL.speedCtrl.kp);
    comsRegisterParam(&coms, P_LKI,     "%f", &motorL.speedCtrl.ki);
    // - RIGHT MOTOR
    comsRegisterParam(&coms, P_RTRGT,   "%f", &motorR.speedCtrl.ref);
    comsRegisterParam(&coms, P_RKP,     "%f", &motorR.speedCtrl.kp);
    comsRegisterParam(&coms, P_RKI,     "%f", &motorR.speedCtrl.ki);
    // - BALANCE CONTROLLER
    comsRegisterParam(&coms, P_BTRGT,   "%f", &balanceAngleCtrl.ref);
    comsRegisterParam(&coms, P_BKP,     "%f", &balanceAngleCtrl.kp);
    comsRegisterParam(&coms, P_BKI,     "%f", &balanceAngleCtrl.ki);
    comsRegisterParam(&coms, P_BKD,     "%f", &balanceAngleCtrl.kd);
    // - Linear Velocity Controller
    comsRegisterParam(&coms, P_VTRGT,   "%f", &velCtrl.ref);
    comsRegisterParam(&coms, P_VKP,     "%f", &velCtrl.kp);
    comsRegisterParam(&coms, P_VKI,     "%f", &velCtrl.ki);
    comsRegisterParam(&coms, P_VKD,     "%f", &velCtrl.kd);
    comsRegisterParam(&coms, P_VAPLHA,  "%f", &ddmr.linearVelAlpha);
    // - Angular Velocity Controller
    comsRegisterParam(&coms, P_ATRGT,   "%f", &steerCtrl.ref);
    comsRegisterParam(&coms, P_AKP,     "%f", &steerCtrl.kp);
    comsRegisterParam(&coms, P_AKI,     "%f", &steerCtrl.ki);
    comsRegisterParam(&coms, P_AKD,     "%f", &steerCtrl.kd);
    comsRegisterParam(&coms, P_AALPHA,  "%f", &ddmr.angularVelAlpha);
    // - IMU Parameters
    comsRegisterParam(&coms, P_IMU_AA,  "%f", &imu.lpf.alpha_accel);
    comsRegisterParam(&coms, P_IMU_GA,  "%f", &imu.lpf.alpha_gyro);
    comsRegisterParam(&coms, P_IMU_KQ,  "%f", &imu.kal.pitchK.Q_angle);
    comsRegisterParam(&coms, P_IMU_KQB, "%f", &imu.kal.pitchK.Q_bias);
    comsRegisterParam(&coms, P_IMU_KR,  "%f", &imu.kal.pitchK.R_measure);
    comsRegisterParam(&coms, P_IMU_A_XOFFSET,  "%f", &imu.sensor->offset.accel.x);
    comsRegisterParam(&coms, P_IMU_A_YOFFSET, "%f", &imu.sensor->offset.accel.y);
    comsRegisterParam(&coms, P_IMU_A_ZOFFSET, "%f", &imu.sensor->offset.accel.z);
    comsRegisterParam(&coms, P_IMU_MOUNT_OFFSET, "%f", &imu.sensor->mount_offset);
    comsRegisterParam(&coms, P_K1, "%f", &lqr.K[0]);
    comsRegisterParam(&coms, P_K2, "%f", &lqr.K[1]);
    comsRegisterParam(&coms, P_K3, "%f", &lqr.K[2]);
    comsRegisterParam(&coms, P_K4, "%f", &lqr.K[3]);

    // ************************************************************** //
    comsSendMsg(&coms, &ser1, PUB_INFO, "POST PASSED");
    // ***** Application Tasks **************************************************************************** // 
    struct FixedTimeTask blinkTask = createTask(BLINK_PERIOD);      // Watchdog led
    struct FixedTimeTask comsCmdTask = createTask(COMSCMD_PERIOD); // SUB RPC 
    struct FixedTimeTask comsPUBTask = createTask(COMSPUB_PERIOD); // PUB Telemetry
    struct FixedTimeTask imuTask =  createTask(IMU_PERIOD); // IMU Read
    struct FixedTimeTask odomTask =  createTask(ODOM_PERIOD); // Odometry
    struct FixedTimeTask wspeedCntlTask = createTask(WSPEED_CNTRL_PERIOD); // Motor Speed Control
    struct FixedTimeTask balanceCntrlTask = createTask(BAL_CNTRL_PERIOD);  // Balance Control Loop
    struct FixedTimeTask velCtrlTask = createTask(VEL_CNTRL_PERIOD);  // Velocity Control Loop
    struct FixedTimeTask lqrTask = createTask(LQR_CNTRL_PERIOD); // LQR Controller


    // ****** Loop Parameters **************************************************************************** // 
    enum MODE {FW_UPDATE=0,INIT,PARK, CASCADE, LQR,  TUNE_MOTOR, COMMISSION, SYSID} mode;
    nextMode = INIT;
    motorEnable(&motorL);
    motorEnable(&motorR);

    uint16_t loopTick = 0;
	for(;;){
        loopTick = sysGetMillis(); // Update Loop Time
        mode = nextMode;
        switch (mode){
            case FW_UPDATE:
                comsSendMsg(&coms, &ser1, PUB_INFO, "FW_UPDATE");
                // Disable Motors, safety disable drivers
                motorDisable(&motorL);
                motorDisable(&motorR);
                // Set GPIO Values High Z
                gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_ALL);
                gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_ALL);
                gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_ALL);
                // Jump to Bootloader
                systemReflash();
                break;
            case INIT: // Initialize System 
                // State Transition
                if(sysGetMillis() >= 3000){ // 3s Startup helps stabilize Kalman Filter
                    systick_clear();
                    pidClear(&velCtrl);
                    pidClear(&balanceAngleCtrl);
                    pidClear(&steerCtrl);
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    motorStop(&motorL);
                    motorStop(&motorR); 
                    nextMode = PARK;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "INIT => PARK ");
                }
                break;
            case PARK:
                // State Transition
                if(_fabs(imu.kal.pitch) <= BAL_CUTOFF){ // Safety Cut Off
                    motorEnable(&motorL);
                    motorEnable(&motorR);
                    pidClear(&velCtrl);
                    pidClear(&balanceAngleCtrl);
                    pidClear(&steerCtrl);
                    ddmrReset(&ddmr);
                    nextMode = CASCADE;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "PARK => RUN");
                }
                break;
            case CASCADE:
                if(_fabs(imu.kal.pitch) >= BAL_CUTOFF){  // Safety Cut Off
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    nextMode = PARK;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "CASCADE => PARK "); 
                    break;
                }
                if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
                    runMotorSpeedLoop(&motorL);
                    runMotorSpeedLoop(&motorR);
                    wspeedCntlTask.lastTick = loopTick;
                }
                if(CHECK_PERIOD(balanceCntrlTask, loopTick)){  
                    runBalanceLoop(&balanceAngleCtrl, &steerCtrl, &imu, &motorL, &motorR);
                    balanceCntrlTask.lastTick = loopTick;
                }
                if(CHECK_PERIOD(velCtrlTask, loopTick)){    
                    runVelocityLoop(&velCtrl, &steerCtrl, &balanceAngleCtrl, &ddmr);
                    velCtrlTask.lastTick= loopTick;
                }
                break;
            case LQR:
                if(_fabs(imu.kal.pitch) >= BAL_CUTOFF){  // Safety Cut Off
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    nextMode = PARK;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "LQR => PARK "); 
                    break;
                }
                if(CHECK_PERIOD(lqrTask, loopTick)){
                    runLQR(&lqr, &motorL, &motorR);
                    lqrTask.lastTick = loopTick;
                }
                break;
            case TUNE_MOTOR:
                motorEnable(&motorL);
                motorEnable(&motorR);
                nextMode = COMMISSION;
                comsSendMsg(&coms, &ser1, PUB_INFO, "TUNE_MOTOR => COMMISSION");
                break;
            case COMMISSION:
                if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
                    runMotorSpeedLoop(&motorL);
                    runMotorSpeedLoop(&motorR);
                    wspeedCntlTask.lastTick = loopTick;
                }
                break;
            case SYSID:
                if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
                    motorSetVoltage(&motorL, motorL.speedCtrl.ref);
                    motorSetVoltage(&motorR, motorR.speedCtrl.ref);
                    wspeedCntlTask.lastTick = loopTick;
                }
                break;
            default:
                nextMode = PARK;
                break;   
        };

        // ********** ODOMETRY ********************************************************************************* //
        if(CHECK_PERIOD(odomTask, loopTick)){
            motorEstSpeed(&motorL);
            motorEstSpeed(&motorR);
            ddmrEstimateOdom(&ddmr, &motorL, &motorR);
            odomTask.lastTick = loopTick;
        }
        // ********** IMU ********************************************************************************* //
        if(CHECK_PERIOD(imuTask, loopTick)){
            mpu6050Read(&mpu6050);
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro);
            imuKalUpdate(&imu);
            imu.kal.pitch += imu.sensor->mount_offset;
            imuTask.lastTick = loopTick;
        }
        // ********** Communications ********************************************************************** //
        if(CHECK_PERIOD(comsCmdTask, loopTick)){
            // Handle RX Messages 
            while(rbEmpty(&ser1.rxRB)==false){
                if(comsGrabCmdMsg(&coms, &ser1)){
                    comsExecuteRPC(&coms, &ser1);
                }
            }
            comsCmdTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(comsPUBTask, loopTick)){
            // TX Publishing Telemetry
            comsSendMsg(&coms, &ser1, PUB_TELEM,    
                imu.kal.pitch,      imu.kal.roll, 
                imu.lpf.accel.x,    imu.lpf.accel.y,        imu.lpf.accel.z,
                imu.lpf.gyro.x,     imu.lpf.gyro.y,         imu.lpf.gyro.z,
                motorL.wheelRPS,    motorL.speedCtrl.ref,   motorL.speedCtrl.out,
                motorR.wheelRPS,    motorR.speedCtrl.ref,   motorR.speedCtrl.out,
                ddmr.v_b,           velCtrl.ref,            balanceAngleCtrl.ref, 
                ddmr.w_b,           steerCtrl.ref,          steerCtrl.out,
                ddmr.posX,          ddmr.x_g,               ddmr.y_g,
                ddmr.psi,           lqr.u);

            comsPUBTask.lastTick = loopTick;
        }

        // ********* DEBUG LED ******************************************************************* // 
        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;  
        }
    }
}