PROJECT_NAME     := uECG5
TARGETS          := uECG5
OUTPUT_DIRECTORY := build

SDK_ROOT := ../urf_lib/nrf_usdk52
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/uECG5.out: \
  LINKER_SCRIPT  := uECG_nrf52.ld 

# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/mcp3911.c \
  $(PROJ_DIR)/leds.c \
  $(PROJ_DIR)/bmi160.c \
  $(PROJ_DIR)/fft_opt.c \
  $(PROJ_DIR)/board_config.c \
  $(PROJ_DIR)/data_packing.c \
  $(PROJ_DIR)/ecg_processor.c \
  $(PROJ_DIR)/imu_processor.c \
  $(PROJ_DIR)/radio_functions.c \
  $(PROJ_DIR)/spim_functions.c \
  $(PROJ_DIR)/../urf_lib/urf_timer.c \
  $(PROJ_DIR)/../urf_lib/urf_radio.c \
  $(PROJ_DIR)/../urf_lib/urf_star_protocol.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_peripheral.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_att_process.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_encryption.c \
  $(PROJ_DIR)/../urf_lib/urf_ble_smp_process.c \
  $(SDK_ROOT)/gcc/gcc_startup_nrf52.S \
  $(SDK_ROOT)/system_nrf52.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../urf_lib \
  $(SDK_ROOT)/device \
  $(SDK_ROOT)/cmsis/include \
  $(SDK_ROOT) \


# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -Os
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums 

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs


# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default

# Default target - first one defined
default: uECG5

TEMPLATE_PATH := $(SDK_ROOT)/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))


