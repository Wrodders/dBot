#ifndef COMMON_H
#define COMMON_H
// ********* REGISTER DEFINITIONS ************************** //
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>
// ********* PERIPHERAL DEFINITIONS ************************ //
#include "pinnmapV1.h"
// ********* COMMON LIBRARIES ****************************** //
#include "../drivers/system.h"
#include "../../modules/cutils/ringbuffer.h"
#include "../../modules/cutils/utils.h"
#include "../../modules/cUtils/printf.h"
#include "../../modules/cutils/queue.h"

// ******************** MACROS **************************** //
#define ARRAY_SIZE(arr) (size_t)(sizeof(arr) / sizeof((arr)[0]))
#ifndef NULL
#define NULL ((void *)0)
#endif
// ************* Task Execution Periods ******************** // 
#define COMSCMD_PERIOD         20  // ms
#define COMSPUB_PERIOD         10  // ms
#define BLINK_PERIOD           200 // ms
#define IMU_PERIOD             1
#define WSPEED_CNTRL_PERIOD    2
#define BAL_CNTRL_PERIOD       5
#define VEL_CNTRL_PERIOD       10
#define LQR_CNTRL_PERIOD       2
// *********** GLOBAL CONSTANTS *************************** //
#define M_PI                3.14159265358979323846f
#define ENC_CPR             12.00f    // Counts per Revolution
#define GEAR_RATIO          20.00f    // 20:1
#define EDGE_NUM            4.00f     // Quaducore Encoder
#define WHEEL_BASE          0.132f    // m
#define WHEEL_RADIUS        0.04f     // m

#define VSYS                10.00f    // Nominal System Voltage [3s Li ion 10.2-12.6V]
#define VMOTOR_MAX          9.00      // Scale duty to nominal motor voltage
#define BAT_CAPACITY        2300      // mAh
#define MOTOR_DEADBAND      0.0f      // 
// *********** UNIT CONVERSIONS *************************** //
const float MOTOR_CPR       = ENC_CPR * GEAR_RATIO * EDGE_NUM;
const float MS_TO_S         = 0.001f; // s
const float S_TO_MS         = 1000;   // ms
const float TICKS_TO_RPS    = (float)(1/(MOTOR_CPR * (WSPEED_CNTRL_PERIOD * MS_TO_S)));
const float MPS_TO_RPS      = (float) 1/(WHEEL_RADIUS*2*M_PI);
const float RPS_TO_MPS      = (float) 2*M_PI*WHEEL_RADIUS; 
const float RAD_TO_DEG      = (float)(180.0f / M_PI); 
const float RPS_TO_RADS     = (float)(2*M_PI);      
const float RPS_MAX         = 10.00f;               // rotations per second
const float VEL_MAX         = RPS_MAX * RPS_TO_MPS; // m/s
// *********** Initial Default Parameters ********************************** //
// IMU Pre-Filtering
#define IMU_A_ACCEL     0.5f
#define IMU_A_GYRO      0.01f
#define IMU_A_COMP      0.05f
#define IMU_KAL_Q       0.001f // smaller the 
#define IMU_KAL_R       0.03f 
#define IMU_Q_BIAS      0.003f
#define IMU_A_XOFFSET   0.036453f
#define IMU_A_YOFFSET   -0.0021066f
#define IMU_A_ZOFFSET   0.133658f

#define LIN_VEL_FILT_ALPHA  0.95f
#define ANG_VEL_FILT_ALPHA  0.95f

// Motor Speed Control
#define SPEED_KP    2.4f
#define SPEED_KI    5.0f
#define SPEED_KD    0.00f
#define SPEED_ALPHA 0.95f
// Balance Control 
#define IMU_MOUNT_OFFSET  1.0f  // deg
#define BAL_CUTOFF  35  
#define BAL_KP      0.24
#define BAL_KI      5
#define BAL_KD      0.0035f
// Linear Velocity Control
#define VEL_P       3.5
#define VEL_I       0.6
#define VEL_D       0.0
#define VEL_ALPHA   1.0
// Angular Velocity Control
#define STEER_KP    0.7
#define STEER_KI    2.0
#define STEER_KD    0.0
#define STEER_ALPHA 1.0

// Full State Feedback Controller
#define LQR_K1      3.3274
#define LQR_K2      14.1156
#define LQR_K3      2.216
#define LQR_K4      -14.7079


// *********** GLOBAL VARIABLES ******************************************** //
static float vbat_val_ = VSYS;     


#endif // COMMON_h