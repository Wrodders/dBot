#ifndef PINMAP_H
#define PINMAP_H

#include <libopencm3/stm32/gpio.h>
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
#define BT_USART		USART6
#define BT_PORT			GPIOA
#define BT_TX			GPIO11
#define BT_RX			GPIO12
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
#define ENC_1_TIM       TIM3
#define ENC_1_AF        GPIO_AF2
#define ENC_1_PORT      GPIOB
#define ENC_1_A         GPIO4   // TIM3 CH1
#define ENC_1_B         GPIO5   // TIM3 CH2

#define ENC_2_TIM       TIM4
#define ENC_2_AF        GPIO_AF2
#define ENC_2_PORT      GPIOB
#define ENC_2_A         GPIO6   // TIM4 CH1
#define ENC_2_B         GPIO7   // TIM4 CH2
// ADC Inputs
#define IPROP_L         GPIO6
#define IPROP_R         GPIO7
#define VBAT_SENSE      GPIO5
#define ADC_PORT        GPIOA

#endif // PINMAP_H