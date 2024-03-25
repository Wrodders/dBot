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
#define DEBUG_PORT		GPIOB
#define DEBUG_RX		GPIO7 
#define DEBUG_TX		GPIO6
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

#define ENC_R_TIM       TIM1
#define ENC_R_AF        GPIO_AF1
#define ENC_R_PORT      GPIOA
#define ENC_R_A         GPIO8   // TIM1 CH1
#define ENC_R_B         GPIO9   // TIM1 CH2


// ************* Task Execution Periods ******************** // 
#define COMS_PERIOD         100 // 10Hz
#define CTRL_PERIOD         10  // 100Hz
#define BLINK_PERIOD        500 // 2Hz



// *********** GLOBAL CONSTANTS *************************** //
#define M_PI 3.14159265358979323846f
#define ENC_CPR             12.00f
#define GEAR_RATIO          20.00f
#define EDGE_NUM            4.00f

#define WHEEL_BASE          0.07f   // m
#define WHEEL_RADIUS        0.035f  // m
#define VBAT_MAX            10.00f   // 2*4.2V
#define BAT_CAPACITY        2300    // mAh

const float MOTOR_CPR = ENC_CPR * GEAR_RATIO * EDGE_NUM;
const float MS_TO_S = 0.001f;
const float S_TO_MS = 1000;
const float TO_INVERSE = 0.1f;
const float TICKS_TO_RPS  = (float)(1/(MOTOR_CPR * (CTRL_PERIOD * MS_TO_S)));
const float MPS_TO_RPS  = (float)1/(WHEEL_RADIUS*2*M_PI);
const float RPS_TO_MPS  = (float) 2*M_PI*WHEEL_RADIUS;

#define RPS_MAX         8.00f  // rps
#define VEL_MAX         RPS_MAX * RPS_TO_MPS

// Speed Control Parameters
#define SPEED_KP  6.0f
#define SPEED_KI  20.0f
#define SPEED_KD  0.0f
#define BETA_SPEED 0.9f

// Balance Control Parameters
#define BAL_THETA   0.0f
#define BAL_KP      3.0f
#define BAL_KI      0.0f
#define BAL_KD      0.0f
// Motion Control Parameters
#define VEL_KP      0.0f
#define VEL_KI      0.0f
#define VEL_KD      0.0f
// *********** GLOBAL VARIABLES **************************** //
static float VBAT_VAL_ = VBAT_MAX;     


#endif // COMMON_h


