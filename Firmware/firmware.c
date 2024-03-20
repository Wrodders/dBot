// Motor Driver Firmware

#include "common/common.h"
#include "common/task.h"

#include "inc/coms.h"
#include "inc/imu.h"
#include "inc/robot.h"

#define ARRAY_SIZE 100 // Adjust array size as needed
#define POPULATION_INTERVAL 5 // Population interval in seconds

double chirp_frequency(double t, double min_freq, double max_freq, double period) {
    // Calculate the chirp frequency based on time
    double slope = (max_freq - min_freq) / period;
    return min_freq + slope * t;
}

// Function to populate the array with a chirp waveform
void populateArray(float *array, double min_freq, double max_freq, double period) {
    double t_increment = period / ARRAY_SIZE;
    double t = 0.0;

    for (int i = 0; i < ARRAY_SIZE; i++) {
        double frequency = chirp_frequency(t, min_freq, max_freq, period);
        double amplitude = 1.0; // Adjust amplitude as needed

        // Calculate the value of the chirp waveform (sine wave)
       double value = amplitude * sin(0.1*t + (0.0f - 0.1f)*t*t*1/(2*0.1));

        // Store the value in the array
        array[i] = (float)value;

        // Increment time
        t += t_increment;
    }
}

float getNextValue(float *array, size_t *index, bool *rampUp) {
    if (*rampUp) {
        if ((*index)++ == ARRAY_SIZE - 1) {
            *rampUp = false;
        }
    } else {
        if ((*index)-- <= 0) {
            *rampUp = true;
        }
    }
    return array[*index];
}


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
    
    // ***** Application Tasks ***** // 
    FixedTimeTask blinkTask = createTask(BLINK_PERIOD); // watchdog led
    FixedTimeTask comsTask = createTask(COMS_PERIOD); // PUB SUB RPC
    FixedTimeTask speedCtrlTask = createTask(SPEEDCTRL_PERIOD); // Motor PI Loop
    FixedTimeTask balanceTask = createTask(BALANCE_PERIOD); // IMU Process Loop
    FixedTimeTask sweepTask = createTask(50); // init at 1Hz
  

    // ****** Loop Parameters ******* // 

    float chip_array[ARRAY_SIZE];

    populateArray(chip_array, MIN_FREQUENCY, MAX_FREQUENCY, CHIRP_PERIOD_S);


    // Define global variables
    float currentTime = 0; // Current time
    float currentFrequency = MIN_FREQUENCY; // Current frequency
    float targetFrequency = MAX_FREQUENCY; // Target frequency
    bool increasing = true; // Flag to indicate whether the frequency is increasing or decreasing
    float sineValue = 0; // Value of the sine function
    static float freq = 0;





    double time =0;
    uint8_t buf[40]; // tx coms buffer
    uint8_t len = 0; // msg len
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
            comsSendMsg(&coms, &ser1, PUB_IMU, freq, imu.roll);
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

            static size_t index = 0;


            // Update current time
            currentTime = loopTick;
            static int step = -3;
            static bool rampUp  = true;
            int dir = rampUp == true ? 1 : -1;
            // Calculate sine value based on the current frequency
            //sineValue = sin(2 * M_PI * freq  - currentTime ) * 0.5; // Convert currentTime to seconds


            sineValue = getNextValue(chip_array, &index, &rampUp)* dir;
            // Normalize sine value to keep amplitude constant

            // Drive robot with sine wave signal
            robotTankDrive(&bot, sineValue, sineValue);

            // Update last tick
            sweepTask.lastTick = loopTick;
        }

        //sineSweepTest(&bot, loopTick);





	}

}



