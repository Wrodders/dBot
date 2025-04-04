#!/bin/bash
#--------------------------------------------------
# @file    rstCycleMCU.sh
# @brief   Hard Resets the STM32 MCU.

# GPIO chip and line configuration
GPIO_CHIP="gpiochip0"
GPIO_LINE=2  # GPIO 02 (BCM pin 2)

# Pull reset line low (active low)
echo "[RST_MCU] GPIO holding reset line low 100ms"
gpioset "$GPIO_CHIP" "$GPIO_LINE"=0
sleep 0.1  

# Release reset line (open-drain behavior)
echo "[RST_MCU] Releasing reset line"
gpioget "$GPIO_CHIP" "$GPIO_LINE"  # Read the line to release it

echo "[RST_MCU] complete."