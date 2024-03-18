PROJECT = firmware
BUILD_DIR = bin

SHARED_DIR = /
CFILES = firmware.c
CFILES += ../modules/cUtils/printf.c
#AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f401cc
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR = libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
