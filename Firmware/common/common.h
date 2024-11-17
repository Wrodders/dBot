#ifndef COMMON_H
#define COMMON_H
// ********* REGISTER DEFINITIONS ************************** //
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>
// ******** 
#include "pinnmap.h"
#include "../drivers/clock.h"
#include "../../modules/cutils/ringbuffer.h"
#include "../../modules/cutils/utils.h"
#include "../../modules/cUtils/printf.h"
#include "../../modules/cutils/queue.h"

// ******************** MACROS **************************** //
#define ARR_SIZE(arr) (size_t)(sizeof(arr) / sizeof((arr)[0]))
#ifndef NULL
#define NULL ((void *)0)
#endif
// ************* Task Execution Periods ******************** // 
#define COMS_PERIOD             50  // 20Hz
#define BLINK_PERIOD            100 // 10Hz
#define IMU_FUSION_PERIOD       2   //  500 Hz
#define BAL_CNTRL_PERIOD        10  // 250 Hz
#define WSPEED_CNTRL_PERIOD     5   // 200 Hz
#define VEL_CNTRL_PERIOD        20  // 50 Hz 
// *********** GLOBAL CONSTANTS *************************** //
#define M_PI                3.14159265358979323846f
#define ENC_CPR             12.00f
#define GEAR_RATIO          20.00f
#define EDGE_NUM            4.00f
#define WHEEL_BASE          0.12f   // m
#define WHEEL_RADIUS        0.04f   // m
#define VBAT_MIN            7.40f   // 
#define BAT_CAPACITY        2300    // mAh
const float MOTOR_CPR       = ENC_CPR * GEAR_RATIO * EDGE_NUM;
const float MS_TO_S         = 0.001f;
const float S_TO_MS         = 1000;
const float TICKS_TO_RPS    = (float)(1/(MOTOR_CPR * (WSPEED_CNTRL_PERIOD * MS_TO_S)));
const float MPS_TO_RPS      = (float) 1/(WHEEL_RADIUS*2*M_PI);
const float RPS_TO_MPS      = (float) 2*M_PI*WHEEL_RADIUS;
const float RAD_TO_DEG      = (float)(180.0f / M_PI);
const float RPS_TO_RADS     = (float)(2*M_PI);
const float RPS_MAX         = 10.00f;  // rps
const float VEL_MAX         = RPS_MAX * RPS_TO_MPS; //m
// *********** Initial Default Parameters ********************************** //
// IMU Filtering
#define IMU_A_ACCEL 0.5f
#define IMU_A_GYRO  0.01f
#define IMU_A_COMP  0.05f
// Motor Speed Control
#define SPEED_KP    3.0f
#define SPEED_KI    0.4f
#define SPEED_KD    0.00f
#define SPEED_ALPHA 0.95f
// Balance Control 
#define BAL_OFFSET  -7.50f  // deg
#define BAL_CUTOFF  35       
#define BAL_KP      0.1
#define BAL_KI      2.2f 
#define BAL_KD      0.0034f
// Linear Velocity Control
#define VEL_P       18  
#define VEL_I       18
#define VEL_D       0.2
#define VEL_ALPHA   1.0
// Angular Velocity Control
#define STEER_KP    1.2
#define STEER_KI    0.5 
#define STEER_KD    0.003
#define STEER_ALPHA 1.0
// *********** GLOBAL VARIABLES **************************** //
static float VBAT_VAL_ = VBAT_MIN;     

#endif // COMMON_h