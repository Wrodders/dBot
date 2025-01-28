// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/MPU6050.h"
#include "inc/serComs.h"
#include "inc/imu.h"
#include "inc/pid.h"
#include "inc/motor.h"

#include "inc/ddmr.h"
#include "drivers/analog.h"


// ********** GLOBAL STATIC BUFFERS ***************************************// ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[128] = {0};
uint8_t tx1_buf_[256] = {0};
float uuid = 3.14159;
// ********** MOTOR CONTROL FUNCTIONS ***************************************//
static void runMotorSpeedLoop(struct Motor *m){
    motorEstSpeed(m);
    m->speedCtrl.out = pidRun(&m->speedCtrl, m->shaftRPS);
    motorSetVoltage(m, m->speedCtrl.out);
}
static void runBalanceLoop(struct PID *balanceAngleCtrl, struct PID *steerCtrl, struct IMU *imu, 
                    struct DiffDriveModel *ddmr, struct Motor *motorL, struct Motor *motorR){
    ddmrEstimateOdom(ddmr, motorL, motorR);
    // error = (Reference - Calibrated Offset) - Estimated Pitch
    balanceAngleCtrl->out = pidRun(balanceAngleCtrl, imu->kal.pitch); // Unicycle Wheel Speed output
    motorSetTrgtSpeed(motorL, (balanceAngleCtrl->out + steerCtrl->out)); // Apply Steering Correction
    motorSetTrgtSpeed(motorR, (balanceAngleCtrl->out - steerCtrl->out)); // Apply Steering Correction
}
static void runVelocityLoop(struct PID *velCtrl,struct PID *steerCtrl, struct PID *balanceAngleCtrl, 
                            struct DiffDriveModel *ddmr, struct Motor *motorL, struct Motor *motorR){
    // Twist Velocity Control Loop
    ddmrEstimateOdom(ddmr, motorL, motorR);
    velCtrl->out = pidRun(velCtrl, ddmr->linearVel); // Reference - odometry velocity 
    steerCtrl->out = pidRun(steerCtrl,ddmr->angularVel); // Reference - odometry angular velocity
    pidSetRef(balanceAngleCtrl, BAL_OFFSET-velCtrl->out); // Sets Balance Reference point to reach desired velocity
}
// ******************************************************************************************************************** //

// ********* SUPER LOOP **************** // 
int main(void){
     float nextMode = 0;
	// ***************************** HARDWARE SETUP ******************************************************** //
	clock_setup();      // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup();    // 1ms Tick
	struct GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);

    // Serial /*
    struct Serial ser1 = serialInit( DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, 
                            NVIC_USART1_IRQ, 
                            rx1_buf_, ARRAY_SIZE(rx1_buf_), 
                            tx1_buf_, ARRAY_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
   
    // IMU 
    i2cInit(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);
    struct MPU6050 mpu6050 = mpu6050Init(IMU_PERIF);

    struct IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (IMU_PERIOD * MS_TO_S));
    imuLinkSensor(&imu, &mpu6050);
   
    // MOTORS 
    struct Encoder encR = encoderInit(ENC_2_TIM, UINT16_MAX, ENC_2_A, ENC_1_PORT, ENC_2_B, ENC_2_PORT, ENC_2_AF);
    struct Encoder encL = encoderInit(ENC_1_TIM, UINT16_MAX, ENC_1_A, ENC_2_PORT, ENC_1_B, ENC_1_PORT, ENC_1_AF);
    struct Motor motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    struct Motor motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA, TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    motorDisable(&motorL); // Ensure Driver is Disabled Before Configuration
    motorDisable(&motorR); //
    motorConfig(&motorL, &encL, VSYS, MOTOR_DEADBAND, true, SPEED_ALPHA, RPS_MAX);
    motorConfig(&motorR, &encR, VSYS, MOTOR_DEADBAND, false,  SPEED_ALPHA, RPS_MAX);
    motorEnable(&motorL);
    motorEnable(&motorR);

    // BALANCE CONTROLLER
    struct PID balanceAngleCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (BAL_CNTRL_PERIOD * MS_TO_S));
    pidSetRef(&balanceAngleCtrl, -7.5f);
    // DIFFERENTIAL DRIVE MODEL (LinVel, AngVel)
    struct DiffDriveModel ddmr = ddmrInit(WHEEL_RADIUS, WHEEL_BASE, VEL_ALPHA, STEER_ALPHA);
    struct PID velCtrl = pidInit(-6, 6, VEL_P, VEL_I, VEL_D, (VEL_CNTRL_PERIOD * MS_TO_S));   
    struct PID steerCtrl = pidInit(-5,5, STEER_KP, STEER_KI, STEER_KD, VEL_CNTRL_PERIOD * MS_TO_S);
   
   // ****************************** TELEMETRY STATE******************************************************** //
    struct TWSBState twsbState = {
        .pitch = &imu.kal.pitch, .roll = &imu.kal.roll,
        .leftShaftRPS = &motorL.shaftRPS, .rightShaftRPS = &motorR.shaftRPS,
        .leftWheelTargetRPS = &motorL.speedCtrl.ref, 
        .rightWheelTargetRPS = &motorR.speedCtrl.ref,
        .voltageLeft = &motorL.speedCtrl.out, .voltageRight = &motorR.speedCtrl.out,

        .linearVelocity = &ddmr.linearVel, .targetLinVel = &velCtrl.ref, .balanceAngle = &balanceAngleCtrl.ref,
        .angularVelocity = &ddmr.angularVel, .targetAngularVel = &steerCtrl.ref, .steerDiff = &steerCtrl.out
    };
    // ***************************** COMMUNICATIONS ******************************************************** // 
    struct Coms coms = comsInit();
    // -------------------- TOPIC PUBLISHERS ---------------------- //
    comsRegisterPub(&coms, PUB_CMD_RET, SERIALIZED_CMDRET_FMT);
    comsRegisterPub(&coms, PUB_ERROR, SERIALIZED_ERROR_FMT);
    comsRegisterPub(&coms, PUB_INFO, SERIALIZED_INFO_FMT);
    comsRegisterPub(&coms, PUB_DEBUG, SERIALIZED_DEBUG_FMT);
    comsRegisterPub(&coms, PUB_STATE, SERIALIZED_STATE_FMT);
    // --------------------PARAMETER REGISTERS ---------------------- //
    comsRegisterParam(&coms, P_ID,      "%0.3f", &uuid);
    comsRegisterParam(&coms, P_MODE,    "%0.3f", &nextMode);
    // - LEFT MOTOR
    comsRegisterParam(&coms, P_LTRGT,   "%0.3f", &motorL.speedCtrl.ref);
    comsRegisterParam(&coms, P_LKP,     "%0.3f", &motorL.speedCtrl.kp);
    comsRegisterParam(&coms, P_LKI,     "%0.3f", &motorL.speedCtrl.ki);
    // - RIGHT MOTOR
    comsRegisterParam(&coms, P_RTRGT,   "%0.3f", &motorR.speedCtrl.ref);
    comsRegisterParam(&coms, P_RKP,     "%0.3f", &motorR.speedCtrl.kp);
    comsRegisterParam(&coms, P_RKI,     "%0.3f", &motorR.speedCtrl.ki);
    // - BALANCE CONTROLLER
    comsRegisterParam(&coms, P_BTRGT,   "%0.3f", &balanceAngleCtrl.ref);
    comsRegisterParam(&coms, P_BKP,     "%0.3f", &balanceAngleCtrl.kp);
    comsRegisterParam(&coms, P_BKI,     "%0.3f", &balanceAngleCtrl.ki);
    comsRegisterParam(&coms, P_BKD,     "%0.3f", &balanceAngleCtrl.kd);
    // - Linear Velocity Controller
    comsRegisterParam(&coms, P_VTRGT,   "%0.3f", &velCtrl.ref);
    comsRegisterParam(&coms, P_VKP,     "%0.3f", &velCtrl.kp);
    comsRegisterParam(&coms, P_VKI,     "%0.3f", &velCtrl.ki);
    comsRegisterParam(&coms, P_VKD,     "%0.3f", &velCtrl.kd);
    comsRegisterParam(&coms, P_VAPLHA,  "%0.3f", &ddmr.linearVelAlpha);
    // - Angular Velocity Controller
    comsRegisterParam(&coms, P_ATRGT,   "%0.3f", &steerCtrl.ref);
    comsRegisterParam(&coms, P_AKP,     "%0.3f", &steerCtrl.kp);
    comsRegisterParam(&coms, P_AKI,     "%0.3f", &steerCtrl.ki);
    comsRegisterParam(&coms, P_AKD,     "%0.3f", &steerCtrl.kd);
    comsRegisterParam(&coms, P_AALPHA,  "%0.3f", &ddmr.angularVelAlpha);
    // ************************************************************** //
    comsSendMsg(&coms, &ser1, PUB_INFO, "POST PASSED");
    // ***** Application Tasks **************************************************************************** // 
    struct FixedTimeTask blinkTask = createTask(BLINK_PERIOD);              // Watchdog led
    struct FixedTimeTask comsTask = createTask(COMS_PERIOD);                // PUB SUB RPC 
    struct FixedTimeTask imuTask =  createTask(IMU_PERIOD);
    struct FixedTimeTask wspeedCntlTask = createTask(WSPEED_CNTRL_PERIOD);      // Motor Speed Control
    struct FixedTimeTask balanceCntrlTask = createTask(BAL_CNTRL_PERIOD);       // Balance Control Loop
    struct FixedTimeTask velCtrlTask = createTask(VEL_CNTRL_PERIOD);            // Velocity Control Loop

    // ****** Loop Parameters **************************************************************************** // 
    enum MODE {INIT = 0,PARK, CASCADE, LQR, TUNE_MOTOR} mode = INIT;
    uint16_t loopTick = 0;
	for(;;){
        loopTick = get_ticks();
        mode = nextMode;
        switch (mode){
            case INIT: // Initialize System 
                // State Transition
                if(get_ticks() >= 3000){ // 3s Delay
                    systick_clear();
                    pidClear(&velCtrl);
                    pidClear(&balanceAngleCtrl);
                    pidClear(&steerCtrl);
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                   
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
                    nextMode = CASCADE;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "PARK => CASCADE");
                }
                break;
            case CASCADE:
                // State Transition 
                if(_fabs(imu.kal.pitch) >= BAL_CUTOFF){  // Safety Cut Off
                    motorDisable(&motorL);
                    motorDisable(&motorR);
                    nextMode = PARK;
                    comsSendMsg(&coms, &ser1, PUB_INFO, "RUN => PARK "); 
                    break;
                }
                // ** WHEEL SPEED CONTROL LOOP //
                if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
                    // Motor Speed Control ISR 
                    runMotorSpeedLoop(&motorL);
                    runMotorSpeedLoop(&motorR);
                    wspeedCntlTask.lastTick = loopTick;
                }
                // ** BALANCE CONTROL LOOP // 
                if(CHECK_PERIOD(balanceCntrlTask, loopTick)){  
                    runBalanceLoop(&balanceAngleCtrl, &steerCtrl, &imu, &ddmr, &motorL, &motorR);
                    balanceCntrlTask.lastTick = loopTick;
                }
                // ** VELOCITY CONTROL LOOP //
                if(CHECK_PERIOD(velCtrlTask, loopTick)){    
                    runVelocityLoop(&velCtrl, &steerCtrl, &balanceAngleCtrl, &ddmr, &motorL, &motorR);
                    velCtrlTask.lastTick= loopTick;
                }
                break;
            case LQR:
                
                break;

            case TUNE_MOTOR:
                if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
                    // Motor Speed Control ISR 
                    runMotorSpeedLoop(&motorL);
                    runMotorSpeedLoop(&motorR);
                    wspeedCntlTask.lastTick = loopTick;
                }
                break;
            default:
                break;  
        };
        // ********* FIXED TIME TASKS ******************************************************************* // 
        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;  
        }
        if(CHECK_PERIOD(imuTask, loopTick)){
            mpu6050Read(&mpu6050);
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro);
            imuKalUpdate(&imu);


            imuTask.lastTick = loopTick;
        }
        // ********** Communications ********************************************************************** //
        if(CHECK_PERIOD(comsTask, loopTick)){
            // TX Publishing Topic
            comsSendMsg(&coms, &ser1, PUB_STATE, imu.kal.pitch,     imu.kal.roll,    
                                                motorL.shaftRPS,    motorL.speedCtrl.ref,  motorL.speedCtrl.out, 
                                                motorR.shaftRPS,    motorR.speedCtrl.ref,  motorR.speedCtrl.out,  
                                                ddmr.linearVel,     velCtrl.ref,           balanceAngleCtrl.ref,
                                                ddmr.angularVel,    steerCtrl.ref,         steerCtrl.out);             
            // Handle RX Messages 
            if(comsGrabCmdMsg(&coms, &ser1)){
                comsProcessCmdMsg(&coms, &ser1);
            }
            comsTask.lastTick = loopTick;
        }
    }
}
