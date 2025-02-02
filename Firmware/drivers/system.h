#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/syscfg.h>

#define TICK			1000     // 1ms
#define CPU_FREQ		84000000 // 84Mhz

// ST LINK Ids Black Pill as STM32F411re Device id 0x0431
#define SYSTEM_MEMORY_ADDRESSS 0x1FFF0000   // 29 Kbytes : AN2606 33.1 Bootloader configuration Table 64. STM32F411xx
#define APPLICATION_ADDRESS 0x08000000      // 1MB Flash

// **** ARM Cortex SysTick Timer **** //

volatile uint32_t _millis = 0;
void sys_tick_handler(void){
	_millis++;
	return;
}

static uint32_t sysGetMillis(void){return _millis;}

static void delay(uint32_t milliseconds) {
    // @Brief Blocking delay
    uint32_t end_time = sysGetMillis() + milliseconds;
    while (sysGetMillis() < end_time) {};
}

static void systick_setup(void){
	systick_set_frequency(TICK, CPU_FREQ); // 1 ms interrupt	
    systick_interrupt_enable();
	systick_counter_enable();
    
}

static void sysTickTaredown(void){                
    SysTick->CTRL = 0; // Disable Systick
    SysTick->VAL = 0;  // Clear Systick Value
    SysTick->LOAD = 0; // Clear Systick Load
}

// ***** System Clock Configuration *********** //



//@Brief: Setup System Peripherals
//@Note: All peripherals used by Drivers need to be enabled
static void systemPeripheralSetup(void){
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);    
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_I2C1);

    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM4);
}

//@Brief: Setup System Clock to 84Mhz using PLL
//
static void systemClockSetup(void){
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    systemPeripheralSetup();
}


//@Brief: Taredown System Peripherals
//@Note: All peripherals used by Drivers need to be disabled
static void systemClockTaredown(void){
    rcc_periph_clock_disable(RCC_GPIOA);
    rcc_periph_clock_disable(RCC_GPIOB);    
    rcc_periph_clock_disable(RCC_GPIOC);

    rcc_periph_clock_disable(RCC_USART1);
    rcc_periph_clock_disable(RCC_I2C1);

    rcc_periph_clock_disable(RCC_TIM2);
    rcc_periph_clock_disable(RCC_TIM3);
    rcc_periph_clock_disable(RCC_TIM4);
}

// ************ System Memory Bootloader Jump ************ //

//@Brief: Set Main Stack Pointer
static inline __attribute__((always_inline)) void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}

//@Brief: Jump to Bootloader in System Memory
//@Note: Requires the all the peripherals to be disabled before jumping
static void systemReflash(void){
    sysTickTaredown(); // Disable Systick
    systemClockTaredown(); // Disable All Peripherals Clocks
    SCB_VTOR = SYSTEM_MEMORY_ADDRESSS; // Set Vector Table to System Memory
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]); // Set System Clock to 16
    __asm__ volatile ("CPSID I\n"); // Disable Interrupts
    // remap the system memory
    SYSCFG_MEMRM = 0x01; // RM0383 rev 3 Section 7.2.1 System configuration controller
    __asm__ volatile ("CPSIE I\n"); // Enable Interrupts
    __set_MSP(*(uint32_t *)SYSTEM_MEMORY_ADDRESSS); // Set Main Stack Pointer
    void (*jumpToBootLoader)(void) = (void (*)(void)) (*((uint32_t *) SYSTEM_MEMORY_ADDRESSS + 1)); //  HACK (FROM STACK OVERFLOW)
    jumpToBootLoader(); // Jump to bootloader @ System Memory Address
}

#endif