#ifndef COMMON_H
#define COMMON_H
// ********* REGISTER DEFINITIONS ************************** //
#include <libopencm3/stm32/rcc.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#include "../drivers/clock.h"

#include "../../modules/cutils/ringbuffer.h"
#include "../../modules/cutils/utils.h"
#include "../../modules/cUtils/printf.h"
#include "../../modules/cutils/queue.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

// ******************** MACROS **************************** //
#define ARR_SIZE(arr) (size_t)(sizeof(arr) / sizeof((arr)[0]))

// **************** PIN DEFINITIONS ********************** //
// Debug Led
#define LED_PORT 		GPIOC
#define LED_PIN			GPIO13
// Debug UART 
#define DEBUG_USART		USART1
#define DEBUG_PORT		GPIOA
#define DEBUG_RX		GPIO10 
#define DEBUG_TX		GPIO9
// Bluetooth UART
#define BT_USART		USART2
#define BT_PORT			GPIOA
#define BT_TX			GPIO2
#define BT_RX			GPIO3
// MPU6050 IMU
#define IMU_PERIF       I2C1
#define IMU_PORT		GPIOB 
#define IMU_SDA			GPIO9
#define IMU_SCL			GPIO8
// DRV8833 2-CH PWM DC Motor Driver 
#define M_L_TIM 	    TIM2    // Left Motor PWM Timer 
#define M_L_PORT	    GPIOA   // Left Motor PWM Port
#define M_L_PWMA        GPIO0   // Left Motor PWM CH1 Pin
#define M_L_PWMB	    GPIO1   // Left Motor PWM CH2 Pin

#define M_R_TIM		    TIM2    // Right Motor PWM Timer 
#define M_R_PORT	    GPIOA   // Right Motor PWM Port
#define M_R_PWMA		GPIO2   // Right Motor PWM CH3 Pin
#define M_R_PWMB		GPIO3   // Right Motor PWM CH4 Pin

#define DRV_EN_PIN      GPIO4   // DRV8833 Enable PIN
#define DRV_EN_PORT     GPIOA   // DRV8833 Enable PORT
// Quaducore Hall Effect Encoder
#define ENC_L_TIM       TIM3
#define ENC_L_AF        GPIO_AF2
#define ENC_L_PORT      GPIOB
#define ENC_L_A         GPIO4   // TIM3 CH1
#define ENC_L_B         GPIO5   // TIM3 CH2

#define ENC_R_TIM       TIM4
#define ENC_R_AF        GPIO_AF2
#define ENC_R_PORT      GPIOB
#define ENC_R_A         GPIO6   // TIM4 CH1
#define ENC_R_B         GPIO7   // TIM4 CH2

// ************* Task Execution Periods ******************** // 
#define COMS_PERIOD         100 // 10Hz
#define CTRL_PERIOD         5  // 100Hz
#define BLINK_PERIOD        500 // 2Hz
#define MCTRL_PERIOD        10 //
// *********** GLOBAL CONSTANTS *************************** //

#define M_PI                3.14159265358979323846f
#define ENC_CPR             12.00f
#define GEAR_RATIO          20.00f
#define EDGE_NUM            4.00f
#define WHEEL_BASE          0.06f   // m
#define WHEEL_RADIUS        0.035f  // m
#define VBAT_MAX            8.40f   // 
#define BAT_CAPACITY        2300    // mAh

const float MOTOR_CPR       = ENC_CPR * GEAR_RATIO * EDGE_NUM;


const float MS_TO_S         = 0.001f;
const float S_TO_MS         = 1000;
const float TICKS_TO_RPS    = (float)(1/(MOTOR_CPR * (CTRL_PERIOD * MS_TO_S)));
const float MPS_TO_RPS      = (float) 1/(WHEEL_RADIUS*2*M_PI);
const float RPS_TO_MPS      = (float) 2*M_PI*WHEEL_RADIUS;
const float RAD_TO_DEG      = (float)(180.0f / M_PI);

const float RPS_MAX         = 8.00f;  // rps
const float VEL_MAX         = RPS_MAX * RPS_TO_MPS;




// *********** Parameters ********************************** //
// IMU Filtering
#define IMU_A_ACCEL 0.5f
#define IMU_A_GYRO  0.01f
#define IMU_A_COMP  0.05f

// Motor Speed Control
#define SPEED_KP    3.0f
#define SPEED_KI    12.0f
#define SPEED_KD    0.00f
#define SPEED_BETA  0.9f
// Balance Control 
#define BAL_OFFSET          -8.0f    // deg
#define BAL_MAX_RECOVERY    30      //
#define BAL_CUTOFF          45      // 
#define BAL_KP              0.18  
#define BAL_KI              1.0f 
#define BAL_KD              0.0045f 
// Motion Control 
#define VEL_KP    2.0f
#define VEL_KI     0.0005f
#define VEL_KD     10.0f
#define VEL_ALPHA  0.5f
// *********** GLOBAL VARIABLES **************************** //
static float VBAT_VAL_ = VBAT_MAX;     

#endif // COMMON_h


