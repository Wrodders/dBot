#ifndef COMMON_H
#define COMMON_H
// ********* REGISTER DEFINITIONS ************************** //
#include <libopencm3/stm32/rcc.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#include "../../modules/cutils/ringbuffer.h"
#include "../../modules/cutils/utils.h"
#include "../../modules/cutils/queue.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

// ******************** MACROS **************************** //
#define ARR_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


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
#define M_L_CH1		    GPIO0   // Left Motor PWM CH1 Pin
#define M_L_CH2		    GPIO1   // Left Motor PWM CH2 Pin

#define M_R_TIM		    TIM2    // Right Motor PWM Timer 
#define M_R_PORT	    GPIOA   // Right Motor PWM Port
#define M_R_CH1		    GPIO2   // Right Motor PWM CH1 Pin
#define M_R_CH2		    GPIO3   // Right Motor PWM CH2 Pin

#define DRV_EN_PIN      GPIO4   // DRV8833 Enable PIN
#define DRV_EN_PORT     GPIOA   // DRV8833 Enable PORT
// Quaducore Hall Effect Encoder
#define ENC_TIM         TIM3
#define ENC_L_PORT      GPIOB
#define ENC_L_A         GPIO4   // TIM3 CH1
#define ENC_L_B         GPIO5   // TIM3 CH2

#define ENC_R_PORT      GPIOB
#define ENC_R_A         GPIO0   // TIM3 CH3
#define ENC_R_B         GPIO1   // TIM3 CH4


// ************* Task Execution Periods ******************** // 
#define COMS_PERIOD         100 // 10Hz
#define SPEEDCTRL_PERIOD    50  // 20Hz
#define BLINK_PERIOD        500 // 2Hz
// *********** GLOBAL CONSTANTS *************************** //
#define ENC_CPR             12.00f
#define GEAR_RATIO          20.00f
#define EDGE_NUM            4.00f
const float MOTOR_CPR = ENC_CPR * GEAR_RATIO * EDGE_NUM;
const float TICKS_TO_RPS  = (float)(1/ MOTOR_CPR) * (1/(SPEEDCTRL_PERIOD * 0.001));

#define WHEEL_BASE      0.07f   // m
#define WHEEL_RADIUS    0.035f  // m
#define VBAT_MAX        8.40f   // 2*4.2V
#define BAT_CAPACITY    2300    // mAh

// *********** GLOBAL VARIABLES **************************** //
static float VBAT_VAL_ = VBAT_MAX;     

// ********** GLOBAL STATIC BUFFERS *************************************** // 
#define RB_SIZE 64          // ACCESS THROUGH RING BUFFER 
uint8_t rx1_buf_[RB_SIZE] = {0};
uint8_t tx1_buf_[RB_SIZE] = {0};


#endif // COMMON_h


