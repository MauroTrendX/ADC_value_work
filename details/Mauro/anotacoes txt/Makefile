PROJECT_NAME := MyWatt_V1

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

SDK_LOC := C:/Users/RAE/Documents/SDK_10

TEMPLATE_PATH = $(SDK_LOC)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath source/main.c) \
$(abspath source/ads018.c) \
$(abspath source/ads018_NV.c) \
$(abspath source/ads1120.c) \
$(abspath source/ant_bpwr_measurement.c) \
$(abspath source/ble_rcs.c) \
$(abspath source/measurement.c) \
$(abspath source/mma8653.c) \
$(abspath custom_lib/ble_cps/source/ble_cps.c) \
$(abspath custom_lib/ble_cps/source/ble_cp_ctrlpt.c) \
$(SDK_LOC)/components/toolchain/system_nrf51.c \
$(SDK_LOC)/components/libraries/util/app_error.c \
$(SDK_LOC)/components/libraries/util/app_util_platform.c \
$(SDK_LOC)/components/libraries/util/nrf_assert.c \
$(SDK_LOC)/components/libraries/uart/retarget.c \
$(SDK_LOC)/components/libraries/uart/app_uart_fifo.c	\
$(SDK_LOC)/components/libraries/fifo/app_fifo.c \
$(SDK_LOC)/components/libraries/trace/app_trace.c \
$(SDK_LOC)/components/libraries/bootloader_dfu/bootloader_util.c \
$(SDK_LOC)/components/libraries/bootloader_dfu/dfu_app_handler.c \
$(SDK_LOC)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_LOC)/components/drivers_nrf/delay/nrf_delay.c \
$(SDK_LOC)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_LOC)/components/drivers_nrf/uart/nrf_drv_uart.c \
$(SDK_LOC)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
$(SDK_LOC)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
$(SDK_LOC)/components/drivers_nrf/timer/nrf_drv_timer.c \
$(SDK_LOC)/components/drivers_nrf/ppi/nrf_drv_ppi.c \
$(SDK_LOC)/components/drivers_nrf/hal/nrf_adc.c \
$(SDK_LOC)/components/ble/common/ble_advdata.c \
$(SDK_LOC)/components/ble/ble_advertising/ble_advertising.c \
$(SDK_LOC)/components/ble/ble_services/ble_bas/ble_bas.c \
$(SDK_LOC)/components/ble/common/ble_conn_params.c \
$(SDK_LOC)/components/ble/ble_services/ble_dfu/ble_dfu.c \
$(SDK_LOC)/components/ble/ble_services/ble_dis/ble_dis.c \
$(SDK_LOC)/components/ble/ble_services/ble_hrs/ble_hrs.c \
$(SDK_LOC)/components/ble/common/ble_srv_common.c \
$(SDK_LOC)/components/ble/device_manager/device_manager_peripheral.c \
$(SDK_LOC)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(SDK_LOC)/components/drivers_nrf/pstorage/pstorage.c	\
$(SDK_LOC)/components/libraries/low_power_pwm/low_power_pwm.c	\
$(SDK_LOC)/components/libraries/button/app_button.c	\
$(SDK_LOC)/components/libraries/timer/app_timer.c	\
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_common_data.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_page_1.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_page_16.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_page_17.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_page_18.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/ant_bpwr_page_torque.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/ant_bpwr.c \
$(SDK_LOC)/components/ant/ant_channel_config/ant_channel_config.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_common/pages/ant_common_page_80.c \
$(SDK_LOC)/components/ant/ant_profiles/ant_common/pages/ant_common_page_81.c \
$(SDK_LOC)/components/ant/ant_key_manager/ant_key_manager.c \
$(SDK_LOC)/components/ant/ant_stack_config/ant_stack_config.c \


#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_LOC)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS += -I$(abspath include)
INC_PATHS += -I$(abspath custom_lib/ble_cps/include)
INC_PATHS += -I$(SDK_LOC)/components/toolchain/gcc
INC_PATHS += -I$(SDK_LOC)/components/toolchain
INC_PATHS += -I$(SDK_LOC)/components/softdevice/s310/headers
INC_PATHS += -I$(SDK_LOC)/examples/bsp
INC_PATHS += -I$(SDK_LOC)/components/device
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/nrf_soc_nosd
INC_PATHS += -I$(SDK_LOC)/components/libraries/uart
INC_PATHS += -I$(SDK_LOC)/components/libraries/util
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/spi_master
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/twi_master
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/timer
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/ppi
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_LOC)/components/libraries/fifo
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_LOC)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_services/ble_hrs
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_LOC)/components/libraries/timer
INC_PATHS += -I$(SDK_LOC)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(SDK_LOC)/components/ble/common
INC_PATHS += -I$(SDK_LOC)/components/ble/device_manager
INC_PATHS += -I$(SDK_LOC)/components/libraries/bootloader_dfu
INC_PATHS += -I$(SDK_LOC)/components/libraries/trace
INC_PATHS += -I$(SDK_LOC)/components/libraries/low_power_pwm
INC_PATHS += -I$(SDK_LOC)/components/libraries/button
INC_PATHS += -I$(SDK_LOC)/components/libraries/bootloader_dfu
INC_PATHS += -I$(SDK_LOC)/components/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_channel_config
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/utils
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_bpwr/pages/logger
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_key_manager
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_key_manager/config
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_common/pages
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_profiles/ant_common/pages/logger
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_stack_config
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_stack_config/config
INC_PATHS += -I$(SDK_LOC)/components/ant/ant_state_indicator
INC_PATHS += -I$(SDK_LOC)/components/libraries/sensorsim


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
#CFLAGS  = -DBOARD_PCA10028
CFLAGS  = -DBOARD_CUSTOM
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DS310
CFLAGS += -DNRF51
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DANT_STACK_SUPPORT_REQD
CFLAGS += -DMODIFICATION_TYPE=0
CFLAGS += -DSENSOR_TYPE=0
CFLAGS += -DTRACE_BPWR_PAGE_1_ENABLE
CFLAGS += -DTRACE_BPWR_PAGE_16_ENABLE
CFLAGS += -DTRACE_BPWR_PAGE_17_ENABLE
CFLAGS += -DTRACE_BPWR_PAGE_18_ENABLE
CFLAGS += -DTRACE_COMMON_PAGE_80_ENABLE
CFLAGS += -DTRACE_COMMON_PAGE_81_ENABLE
CFLAGS += -DTRACE_BPWR_GENERAL_ENABLE
CFLAGS += -DBLE_DFU_APP_SUPPORT
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O0 -g3 -fomit-frame-pointer
#CFLAGS += -Wall -Wno-unused-variable -O0 -g3 -fomit-frame-pointer
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
#ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DS310
ASMFLAGS += -DNRF51
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DANT_STACK_SUPPORT_REQD
ASMFLAGS += -DMODIFICATION_TYPE=0
ASMFLAGS += -DSENSOR_TYPE=0
ASMFLAGS += -DTRACE_BPWR_PAGE_1_ENABLE
ASMFLAGS += -DTRACE_BPWR_PAGE_16_ENABLE
ASMFLAGS += -DTRACE_BPWR_PAGE_17_ENABLE
ASMFLAGS += -DTRACE_BPWR_PAGE_18_ENABLE
ASMFLAGS += -DTRACE_COMMON_PAGE_80_ENABLE
ASMFLAGS += -DTRACE_COMMON_PAGE_81_ENABLE
ASMFLAGS += -DTRACE_BPWR_GENERAL_ENABLE
ASMFLAGS += -DBLE_DFU_APP_SUPPORT
#default target - first one defined
default: clean MyWatt_V1

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e MyWatt_V1

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	MyWatt_V1
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# use math lib
LIBS = "$(GNU_INSTALL_ROOT)/$(GNU_PREFIX)/lib/thumb/libm.a"

MyWatt_V1: OUTPUT_FILENAME := MyWatt_V1
MyWatt_V1: LINKER_SCRIPT=gcc_nrf51.ld
MyWatt_V1: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset

## Flash softdevice
flash_softdevice: $(MAKECMDGOALS)
	@echo Flashing: s310_nrf51422_3.0.0_softdevice.hex
	nrfjprog --program $(SDK_LOC)/components/softdevice/s310/hex/s310_nrf51422_3.0.0_softdevice.hex -f nrf51 --sectorerase
	nrfjprog --reset
#	@echo Flashing: s110_nrf51_8.0.0_softdevice.hex
#	nrfjprog --program $(SDK_LOC)/components/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex -f nrf51 --chiperase
#	nrfjprog --reset
	
	