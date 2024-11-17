// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/MPU6050.h"
#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/pid.h"
#include "inc/motor.h"

#include "inc/ddmr.h"
#include "drivers/analog.h"

// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 128          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};
float UUID = 3.14159;

// ********* SUPER LOOP **************** // 
int main(void){
    enum STATE {INIT = 0, PARK, RUN} state = INIT;
    enum MODE {MCNTRL = 0, PIDCNTRL,LQRCNTRL} mode = PIDCNTRL;
	// ***************************** HARDWARE SETUP ******************************************************** //
	clock_setup();      // Main System external XTAL 25MHz Init Peripheral Clocks
	systick_setup();    // 1ms Tick
	GPIO led = initGPIO(GPIO13, GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
    // Serial
    Serial ser1 = serialInit(DEBUG_USART, DEBUG_PORT, DEBUG_RX, DEBUG_TX, GPIO_AF7, 
                            NVIC_USART1_IRQ, 
                            rx1_buf_, ARR_SIZE(rx1_buf_), 
                            tx1_buf_, ARR_SIZE(tx1_buf_));
    serialConfig(&ser1, 115200, 8, 1, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    // IMU 
    i2cInit(IMU_PERIF, IMU_PORT, IMU_SCL, IMU_SDA);
    MPU6050 mpu6050 = mpu6050Init(IMU_PERIF);
    IMU imu = imuInit(IMU_A_ACCEL, IMU_A_GYRO,IMU_A_COMP, (IMU_FUSION_PERIOD * MS_TO_S));
    // MOTORS 
    Encoder encR = encoderInit(ENC_2_TIM, UINT16_MAX, ENC_2_A, ENC_1_PORT, ENC_2_B, ENC_2_PORT, ENC_2_AF);
    Encoder encL = encoderInit(ENC_1_TIM, UINT16_MAX, ENC_1_A, ENC_2_PORT, ENC_1_B, ENC_1_PORT, ENC_1_AF);
    Motor motorL = motorInit(M_L_TIM, M_L_PORT, TIM_OC1, M_L_PWMA, TIM_OC2, M_L_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    Motor motorR = motorInit(M_R_TIM, M_R_PORT, TIM_OC3, M_R_PWMA,TIM_OC4, M_R_PWMB, DRV_EN_PIN, DRV_EN_PORT);
    motorDisable(&motorL); // Ensure Driver is Disabled Before Configuration
    motorDisable(&motorR); //
    motorConfig(&motorL, &encL, VBAT_MIN, 1.2f, true, SPEED_ALPHA, RPS_MAX);
    motorConfig(&motorR, &encR, VBAT_MIN, 1.2f, false,  SPEED_ALPHA, RPS_MAX);
    motorEnable(&motorL);
    motorEnable(&motorR);
    // BALANCE CONTROLLER
    PID balanceCtrl = pidInit(-RPS_MAX, RPS_MAX, BAL_KP, BAL_KI, BAL_KD, (BAL_CNTRL_PERIOD * MS_TO_S));
    pidSetRef(&balanceCtrl, -7.5f);
    // DIFFERENTIAL DRIVE MODEL (LinVel, AngVel)
    DDMR ddmr = ddmrInit(WHEEL_RADIUS, WHEEL_BASE, VEL_ALPHA, AA);
    PID velCtrl = pidInit(-6, 6, VEL_P, VEL_I, VEL_D, (VEL_CNTRL_PERIOD * MS_TO_S));   
    PID steerCtrl = pidInit(-5,5, AP, AI, AD, VEL_CNTRL_PERIOD * MS_TO_S);
    // ***************************** COMMUNICATIONS ******************************************************** // 
    const Topic pubMap[NUM_PUBS] = {
        {.id = PUB_CMD_RET, .name = "CMD_RET", .format ="%s:%s", .nArgs=2}, // CMD_ID : RET_VAL
        {.id = PUB_ERROR,   .name = "ERROR",   .format = "%s",   .nArgs=1}, // Error Message
        {.id = PUB_INFO,    .name = "INFO",    .format = "%s",   .nArgs=1}, // System INFO
        {.id = PUB_DEBUG,   .name = "DEBUG",   .format = "%s",   .nArgs=1}, // Debug prints 
        // Application Publisher Topics 
        {.id = PUB_IMU,  .name = "IMU",  .format = "%0.3f:%0.3f", .nArgs=2}, // PITCH:ROLL
        {.id = PUB_ODOM, .name = "ODOM", .format = "%0.3f:%0.3f:%0.3f:%0.3f", .nArgs=4}, //LSPEED:RSPEED:LINVEL:ANGVEL
        {.id = PUB_CTRL, .name = "CTRL", .format = "%0.3f:%0.3f:%0.3f:%0.3f", .nArgs=4} //TL:TR:UL:UR:UB:UA:UV
    };
    const Param paramMap[NUM_PARAMS] = {
        {.id = PRM_ID,   .name = "ID",    .param = &UUID,             .format = "%f"},
        {.id = PRM_LT,   .name = "LT",    .param = &motorL.wCtrl.ref, .format = "%f"},
        {.id = PRM_LP,   .name = "LP",    .param = &motorL.wCtrl.kp,  .format = "%f"},
        {.id = PRM_LI,   .name = "LI",    .param = &motorL.wCtrl.ki,  .format = "%f"},
        {.id = PRM_RT,   .name = "RT",    .param = &motorR.wCtrl.ref, .format = "%f"},
        {.id = PRM_RP,   .name = "RP",    .param = &motorR.wCtrl.kp,  .format = "%f"},
        {.id = PRM_RI,   .name = "RI",    .param = &motorR.wCtrl.ki,  .format = "%f"},
        {.id = PRM_BP,   .name = "BT",    .param = &balanceCtrl.ref,  .format = "%f"},
        {.id = PRM_BP,   .name = "BP",    .param = &balanceCtrl.kp,   .format = "%f"},
        {.id = PRM_BI,   .name = "BI",    .param = &balanceCtrl.ki,   .format = "%f"},
        {.id = PRM_BD,   .name = "BD",    .param = &balanceCtrl.kd,   .format = "%f"},
        {.id = PRM_VT,   .name = "VT",    .param = &velCtrl.ref,      .format = "%f"},
        {.id = PRM_VP,   .name = "VP",    .param = &velCtrl.kp,       .format = "%f"},
        {.id = PRM_VI,   .name = "VI",    .param = &velCtrl.ki,       .format = "%f"},
        {.id = PRM_VD,   .name = "VD",    .param = &velCtrl.kd,       .format = "%f"},
        {.id = PRM_VA,   .name = "VA",    .param = &ddmr.linAlpha,    .format = "%f"},
        {.id = PRM_AT,   .name = "AT",    .param = &steerCtrl.ref,    .format = "%f"},
        {.id = PRM_AP,   .name = "AP",    .param = &steerCtrl.kp,     .format = "%f"},
        {.id = PRM_AI,   .name = "AI",    .param = &steerCtrl.ki,     .format = "%f"},
        {.id = PRM_AD,   .name = "AD",    .param = &steerCtrl.kd,     .format = "%f"},
        {.id = PRM_AA,   .name = "AA",    .param = &ddmr.angAlpha,    .format = "%f"},
    };
    Coms coms = comsInit(pubMap, paramMap);
    comsSendMsg(&coms, &ser1, PUB_INFO, "POST PASSED");

    // ***** Application Tasks **************************************************************************** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD);              // Watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD);                // PUB SUB RPC 
    FixedTimeTask imuFusionTask = createTask(IMU_FUSION_PERIOD);     // Sensor Fusion 
    FixedTimeTask wspeedCntlTask = createTask(WSPEED_CNTRL_PERIOD);  // DC Motor Speed Control (PI)
    FixedTimeTask balanceCntrlTask = createTask(BAL_CNTRL_PERIOD);   // Balance Theta Control
    FixedTimeTask velCtrlTask = createTask(VEL_CNTRL_PERIOD);        // Linear Velocity Control 
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
                    break;
                }
                if(CHECK_PERIOD(balanceCntrlTask, loopTick)){
                    // Balance Control Inverted Pendulum PID
                    balanceCtrl.out = pidRun(&balanceCtrl, imu.kal.pitch); 
                    motorSetTrgtSpeed(&motorR, balanceCtrl.out - steerCtrl.out);
                    motorSetTrgtSpeed(&motorL, balanceCtrl.out + steerCtrl.out);
                    balanceCntrlTask.lastTick = loopTick;
                }
                if(CHECK_PERIOD(velCtrlTask, loopTick)){
                    ddmrEstimateOdom(&ddmr, &motorL, &motorR);
                    velCtrl.out = pidRun(&velCtrl, ddmr.linVel);
                    steerCtrl.out = pidRun(&steerCtrl,ddmr.angVel);
                    pidSetRef(&balanceCtrl, -7.5 -velCtrl.out); // Sets Balance Reference point to reach desired velocity
                    velCtrlTask.lastTick= loopTick;
                }
                break;
            default:
                break;  
        };
        // ********* FIXED TIME TASKS ******************************************************************* // 
        if(CHECK_PERIOD(imuFusionTask, loopTick)){
            mpu6050Read(&mpu6050); // Read Sensor need to move this to Interrupt Based 
            imuLPF(&imu, &mpu6050.accel, &mpu6050.gyro); // Apply Low pass filter
            imuKalUpdate(&imu); // Estimate Angle with Kalman Observer
            imuFusionTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(wspeedCntlTask, loopTick)){
            // Motor Speed Control ISR 
            motorSpeedCtrl(&motorL);
            motorSpeedCtrl(&motorR);
            wspeedCntlTask.lastTick = loopTick;
        }
        if(CHECK_PERIOD(blinkTask, loopTick)){
            gpio_toggle(led.port, led.pin);
            blinkTask.lastTick = loopTick;  
        }
        // ********** Communications ********************************************************************** //
        if(CHECK_PERIOD(comsTask, loopTick)){
            // TX Publishing Topics
            comsSendMsg(&coms, &ser1, PUB_IMU,imu.kal.pitch, imu.kal.roll);
            comsSendMsg(&coms, &ser1, PUB_ODOM, motorL.angularVel, motorR.angularVel, 
                                               ddmr.linVel, ddmr.angVel);
            comsSendMsg(&coms,&ser1, PUB_CTRL, motorL.wCtrl.ref, motorR.wCtrl.ref,
                                                motorL.wCtrl.out,motorR.wCtrl.out,
                                                balanceCtrl.out, steerCtrl.out, velCtrl.out); //TL:TR:UL:UR:UB:UA:UV
            // Handle RX Messages 
            if(comsGrabMsg(&coms, &ser1)){
                comsProcessMsg(&coms, &ser1);
            }
            comsTask.lastTick = loopTick;
        }
    }
}
