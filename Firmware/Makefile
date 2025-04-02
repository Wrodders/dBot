PROJECT = firmware
BUILD_DIR = bin

SHARED_DIR = /
CFILES = firmware.c
CFILES += ../modules/cUtils/printf.c
#AFILES += api-asm.S

# Target device (default to stm32f401cc)
TARGET ?= stm32f401cc

# Set DEVICE and OOCD_FILE based on TARGET
ifeq ($(TARGET),stm32f401cc)
    DEVICE = stm32f401cc
    OOCD_FILE = board/stm32f4discovery.cfg
else ifeq ($(TARGET),stm32f411re)
    DEVICE = stm32f411re
    OOCD_FILE = board/stm32f4discovery.cfg  # Update this if you have a different OOCD file for stm32f411re
else
    $(error Unsupported TARGET: $(TARGET))
endif

# Adjust BUILD_DIR to include the target device name
BUILD_DIR := $(BUILD_DIR)/$(TARGET)

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR = libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk