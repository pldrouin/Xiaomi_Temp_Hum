###############################################################################
#
# MODULE:   Makefile
#
# DESCRIPTION: Makefile for the End Device
# 
 ############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
# You, and any third parties must reproduce the copyright and warranty notice
# and any other legend of ownership on each copy or partial copy of the
# software.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright NXP B.V. 2017. All rights reserved
#
############################################################################

# Application target name

TARGET = EndDevice

###############################################################################
#User definable make parameters that may be overwritten from the command line

# Default target device is the JN5169

JENNIC_CHIP ?= JN5179
JENNIC_CHIP_FAMILY ?=JN517x
JENNIC_PCB             ?= DEVKIT5

STACK_SIZE = 5000
MINIMUM_HEAP_SIZE = 2000

ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
JENNIC_SDK             = JN-SW-4270
ZNCLKCMD               = AppBuildZBPro.ld
JENNIC_PCB             = DEVKIT5
ENDIAN                 = LITTLE_ENDIAN_PROCESSOR
DISABLE_LTO            = 1
endif 
ifeq ($(JENNIC_CHIP_FAMILY), JN516x)
JENNIC_SDK              = JN-SW-4170
ZNCLKCMD                = AppBuildZBPro.ld
JENNIC_PCB             = DEVKIT4
ENDIAN 	               = BIG_ENDIAN
DISABLE_LTO            = 1
endif

###############################################################################
# Network settings

# Optional network.mk for channel
sinclude ../../../network.mk

# Channel (0 for default channels)
SINGLE_CHANNEL ?= 0
CFLAGS         += -DSINGLE_CHANNEL=$(SINGLE_CHANNEL)

###############################################################################
# Default DK4 development kit target hardware

JENNIC_PCB ?= DEVKIT4

DR ?= 

ifeq ($(DR), DR1199)
CFLAGS  += -DBUTTON_MAP_$(DR)
CFLAGS += -D$(DR)
else ifeq ($(DR), DONGLE)
CFLAGS  += -DBUTTON_MAP_DR1199
CFLAGS += -D$(DR)
endif

SERIAL_COMMS ?= 0

ifeq ($(SERIAL_COMMS), 1)
CFLAGS += -DSERIAL_COMMS
endif

# NFC NCI ICODE (new) functionality
APP_NCI_ICODE ?= 0
ifeq ($(DR), DONGLE)
APP_NCI_ICODE = 0
endif
ifeq ($(APP_NCI_ICODE), 1)
CFLAGS  += -DAPP_NCI_ICODE
endif

# NFC NCI AES (old) functionality (6x only)
APP_NCI_AES ?= 0
ifeq ($(DR), DONGLE)
APP_NCI_AES = 0
endif
ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
APP_NCI_AES = 0
endif
ifeq ($(APP_NCI_AES), 1)
CFLAGS  += -DAPP_NCI_AES
endif

#Always tell any actual drivers they're running under an RTOS in this usage
#CFLAGS  += -DRTOS

CFLAGS += -DSLEEPY_ZED

# NFC NTAG ICODE (new) functionality
APP_NTAG_ICODE ?= 0
ifeq ($(APP_NTAG_ICODE), 1)
CFLAGS  += -DAPP_NTAG_ICODE
endif

# NFC NTAG AES (old) functionality
APP_NTAG_AES ?= 0
ifeq ($(APP_NTAG_AES), 1)
CFLAGS  += -DAPP_NTAG_AES
endif

###############################################################################
# Select the network stack (e.g. MAC, ZBPro, ZCL)

JENNIC_STACK ?= ZCL
JENNIC_MAC ?= MiniMacShim

###############################################################################
# ZBPro Stack specific options
ZBPRO_DEVICE_TYPE ?= ZED
#PDM_BUILD_TYPE ?= _EXTERNAL_FLASH
PDM_BUILD_TYPE ?=_EEPROM
BAUD                   ?= 115200
DEBUG                  ?= NONE


# BDB features Enable as required
#BDB_SUPPORT_TOUCHLINK_INITIATOR_END_DEVICE ?= 1
#BDB_SUPPORT_TOUCHLINK_INITIATOR_ROUTER ?= 1
#BDB_SUPPORT_TOUCHLINK_TARGET ?= 1
BDB_SUPPORT_NWK_STEERING ?= 1
#BDB_SUPPORT_NWK_FORMATION ?= 1
BDB_SUPPORT_FIND_AND_BIND_INITIATOR ?= 1
#BDB_SUPPORT_FIND_AND_BIND_TARGET ?= 1
ifeq ($(APP_NTAG_ICODE), 1)
BDB_SUPPORT_OOBC ?= 1
endif
ifeq ($(APP_NTAG_AES), 1)
BDB_SUPPORT_OOBC ?= 1
endif

###############################################################################
# Debug options define DEBUG either HW or SW
#DEBUG ?=SW
#DEBUG ?=HW
#
# Define which UART to use for HW debug
# DEBUG_PORT ?= UART1
#  DEBUG_MODE  ?=  SWD    # For ARM core

CFLAGS += -DUART=0

CFLAGS  += -DUART_BAUD_RATE=$(BAUD)

###############################################################################
# Define TRACE to use with DBG module
TRACE ?=0
ifeq ($(TRACE), 1)
CFLAGS += -DDBG_ENABLE
#CFLAGS += -DDEBUG_921600

# Enable any debug output here:
CFLAGS += -DDEBUG_APP
CFLAGS += -DDEBUG_APP_BDB
CFLAGS += -DDEBUG_APP_EVENT
#CFLAGS += -DDEBUG_BDB
CFLAGS += -DNXP_HARDWARE
CFLAGS += -DDEBUG_ZCL
CFLAGS +=  -DDEBUG_EXCEPTIONS
#CFLAGS += -DDEBUG_CLD_IDENTIFY 
#CFLAGS += -DDEBUG_CLD_ONOFF
#CFLAGS += -DDEBUG_CLD_GROUPS
#CFLAGS  += -DSTRICT_PARAM_CHECK
CFLAGS += -DDEBUG_APP
CFLAGS += -DDEBUG_APP_BUTTON
CFLAGS += -DDEBUG_APP_NTAG
CFLAGS += -DDEBUG_APP_ICODE
CFLAGS += -DDEBUG_APP_EVENT
CFLAGS += -DDEBUG_APP_BDB
CFLAGS += -DDEBUG_APP_NCI
CFLAGS += -DDEBUG_SLEEP
#CFLAGS += -DDEBUG_SERIAL
CFLAGS += -DDEBUG_SI
CFLAGS += -DDEBUG_POLL_SLEEP
CFLAGS += -DDEBUG_REPORT
endif

CFLAGS += -D$(TARGET)

GROUPS ?=0
ifeq ($(GROUPS), 1)
CFLAGS  += -DUSE_GROUPS
CFLAGS += -DGROUP_ID=0x8765
endif

ICODES ?=0
ifeq ($(ICODES), 1)
CFLAGS  += -DBDB_JOIN_USES_INSTALL_CODE_KEY=1
endif

ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
INTERRUPT_HANDLER_BBC = zps_isrMAC
LDFLAGS += -Wl,-u$(INTERRUPT_HANDLER_BBC) -Wl,-defsym,vAHI_IntHandlerBbc_select=$(INTERRUPT_HANDLER_BBC)
endif

###############################################################################
# Path definitions

# Use if application directory contains multiple targets
SDK_BASE_DIR        = $(abspath ../../../../sdk/$(JENNIC_SDK))
APP_BASE            = $(abspath ../..)
APP_BLD_DIR         = $(APP_BASE)/$(TARGET)/Build
APP_SRC_DIR         = $(APP_BASE)/$(TARGET)/Source
APP_COMMON_SRC_DIR  = $(APP_BASE)/Common/Source
HW_SRC_DIR			= $(COMPONENTS_BASE_DIR)/HardwareAPI/Source

###############################################################################
# Application Source files

# Note: Path to source file is found using vpath below, so only .c filename is required
APPSRC = app_main.c
ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
APPSRC += port_JN517x.c
endif
ifeq ($(JENNIC_CHIP_FAMILY), JN516x)
ifeq ($(SERIAL_COMMS), 1)
APPSRC += irq_JN516x.S
else
APPSRC += irq_JN516x_noserial.S
endif
APPSRC += portasm_JN516x.S
APPSRC += port_JN516x.c
#APPSRC += StackMeasure.c
endif
ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
APPSRC += AHI_ModuleConfiguration.c 
endif

ifeq ($(SERIAL_COMMS), 1)
APPSRC += uart.c
APPSRC += app_serial_commands.c
endif

APPSRC += pdum_gen.c
APPSRC += pdum_apdu.S
APPSRC += zps_gen.c

#APPSRC += app_exceptions.c
APPSRC += app_start.c
APPSRC += app_end_device_node.c
APPSRC += app_zcl_task.c
APPSRC += app_buttons.c
APPSRC += app_sht3x.c
APPSRC += app_batt.c
APPSRC += app_led.c
ifeq ($(APP_NTAG_ICODE),1)
APPSRC += app_ntag_icode.c
#APPSRC += app_icode.c
endif
ifeq ($(APP_NTAG_AES),1)
APPSRC += app_ntag_aes.c
endif

APP_ZPSCFG = app.zpscfg

OPTIONAL_STACK_FEATURES = $(shell $(ZPSCONFIG) -n $(TARGET) -f $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) -y )

# Construct binary features list 

ifeq ($(APP_NTAG_ICODE),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_NtagIcode
endif

ifeq ($(APP_NTAG_AES),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_NtagAes
endif

# Construct binary hardware list

TARGET_HARDWARE:=_$(JENNIC_CHIP)_$(DR)$(VARIANT)

ifneq ($(SINGLE_CHANNEL), 0)
TARGET_HARDWARE:=$(TARGET_HARDWARE)_CH$(SINGLE_CHANNEL)
endif

###############################################################################
# Standard Application header search paths

INCFLAGS += -I$(APP_SRC_DIR)
INCFLAGS += -I$(APP_SRC_DIR)/..
INCFLAGS += -I$(APP_COMMON_SRC_DIR)

# Application specific include files
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZCL/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZCIF/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Xcv/Include/
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/Recal/Include/
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/MicroSpecific/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/HardwareAPI/Include

###############################################################################
# Application libraries
# Specify additional Component libraries

#APPLIBS += 

###############################################################################
# NTAG ICODE library
ifeq ($(APP_NTAG_ICODE),1)
# NTAG library folder
APP_NTAG_LIB_DIR  = $(APP_BASE)/NFC
# NTAG library
LDLIBS           += NTAG_$(JENNIC_CHIP_FAMILY)
# NTAG library path
LDFLAGS          += -L$(APP_NTAG_LIB_DIR)/Library
# NTAG include path
INCFLAGS         += -I$(APP_NTAG_LIB_DIR)/Include
endif

###############################################################################
# NTAG AES library
ifeq ($(APP_NTAG_AES),1)
# NTAG library folder
APP_NTAG_LIB_DIR  = $(APP_BASE)/NTAG
# NTAG library
LDLIBS           += NTAG_$(JENNIC_CHIP_FAMILY)
# NTAG library path
LDFLAGS          += -L$(APP_NTAG_LIB_DIR)/Library
# NTAG include path
INCFLAGS         += -I$(APP_NTAG_LIB_DIR)/Include
endif

###############################################################################

# You should not need to edit below this line

###############################################################################
APP_CLUSTERS_HVAC_SRC                       ?= 1
APP_CLUSTERS_MEASUREMENT_AND_SENSING_SRC	?= 1
###############################################################################
# Configure for the selected chip or chip family

include $(SDK_BASE_DIR)/Chip/Common/Build/config.mk
include $(SDK_BASE_DIR)/Platform/Common/Build/config.mk
include $(SDK_BASE_DIR)/Stack/Common/Build/config.mk
include $(SDK_BASE_DIR)/Components/BDB/Build/config.mk

###############################################################################

TEMP = $(APPSRC:.c=.o)
APPOBJS_TMP = $(TEMP:.S=.o)
APPOBJS := $(addprefix $(APP_BLD_DIR)/,$(APPOBJS_TMP))

###############################################################################
# Application dynamic dependencies

APPDEPS_TMP = $(APPOBJS_TMP:.o=.d)
APPDEPS := $(addprefix $(APP_BLD_DIR)/,$(APPDEPS_TMP))

###############################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

APPLDLIBS := $(foreach lib,$(APPLIBS),$(if $(wildcard $(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(addsuffix _$(JENNIC_CHIP).a,$(lib)))),$(addsuffix _$(JENNIC_CHIP),$(lib)),$(addsuffix _$(JENNIC_CHIP_FAMILY),$(lib))))
LDLIBS := $(APPLDLIBS) $(LDLIBS)
LDLIBS += JPT_$(JENNIC_CHIP)

###############################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source 
vpath % $(APP_SRC_DIR):$(APP_COMMON_SRC_DIR):$(ZCL_SRC_DIRS):$(ZCL_SRC):$(BDB_SRC_DIR):$(HW_SRC_DIR)


all: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin

ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -e $(ENDIAN) -f $< -o $(APP_SRC_DIR)
	@echo

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH) -e $(ENDIAN) -f $< -o $(APP_SRC_DIR)
endif

ifeq ($(JENNIC_CHIP_FAMILY),JN516x)
$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -f $< -o $(APP_SRC_DIR)
	@echo

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH) -f $< -o $(APP_SRC_DIR)
endif
	
$(APP_BLD_DIR)/pdum_gen.o: CFLAGS += -fno-lto
$(APP_BLD_DIR)/zps_gen.o: CFLAGS += -fno-lto	


$(APP_BLD_DIR)/%.o: %.S
	$(info Assembling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_BLD_DIR)/$*.d -MP
	@echo

$(APP_BLD_DIR)/%.o: %.c 
	$(info Compiling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_BLD_DIR)/$*.d -MP
	@echo

$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf: $(APPOBJS) $(addsuffix.a,$(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(APPLDLIBS))) 
	$(info Linking $@ ...)
	$(CC) -Wl,--gc-sections -Wl,-u_AppColdStart -Wl,-u_AppWarmStart $(LDFLAGS) -L $(SDK_BASE_DIR)/Stack/ZCL/Build/ -T$(ZNCLKCMD) -o $@ -Wl,--start-group $(APPOBJS) $(addprefix -l,$(LDLIBS)) -lm -Wl,--end-group -Wl,-Map,$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).map 
	$(SIZE) $@ 
	date +%c >> $(APP_BASE)/Doc/size.txt
	$(SIZE) $@ >> $(APP_BASE)/Doc/size.txt

ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf
	$(info Generating binary ...)
	$(OBJCOPY) -S -O binary $< $@
else
$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf
	$(info Generating binary ...)
	$(OBJCOPY) -j .version -j .bir -j .flashheader -j .vsr_table -j .vsr_handlers -j .rodata -j .text -j .data -j .bss -j .heap -j .stack -S -O binary $< $@
endif

###############################################################################

clean:
	rm -f $(APPOBJS) $(APPDEPS) $(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin $(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf $(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).map
	rm -f $(APP_SRC_DIR)/pdum_gen.* $(APP_SRC_DIR)/zps_gen*.*

###############################################################################
