######################################
# target
######################################
TARGET = Citofono


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/stm32f4xx_it.c \
Src/system_stm32f4xx.c \
Src/stm32f4_discovery.c \
Src/cs43l22.c \
Src/stm32f4_discovery_audio.c \
Src/usbh_conf.c \
Src/usbh_diskio_dma.c \
HAL/Src/stm32f4xx_hal_gpio.c \
HAL/Src/stm32f4xx_hal.c \
HAL/Src/stm32f4xx_hal_spi.c \
HAL/Src/stm32f4xx_hal_cortex.c \
HAL/Src/stm32f4xx_hal_dma.c \
HAL/Src/stm32f4xx_hal_rcc.c \
HAL/Src/stm32f4xx_hal_rcc_ex.c \
HAL/Src/stm32f4xx_hal_tim.c \
HAL/Src/stm32f4xx_hal_tim_ex.c \
HAL/Src/stm32f4xx_hal_adc.c \
HAL/Src/stm32f4xx_hal_i2s.c \
HAL/Src/stm32f4xx_hal_i2s_ex.c \
HAL/Src/stm32f4xx_hal_i2c.c \
HAL/Src/stm32f4xx_hal_hcd.c \
HAL/Src/stm32f4xx_ll_usb.c \
FatFs/src/ff.c \
FatFs/src/diskio.c \
FatFs/src/ff_gen_drv.c \
FatFs/src/option/syscall.c \
FatFs/src/option/ccsbcs.c \
STM32_USB_Host_Library/Core/Src/usbh_core.c \
STM32_USB_Host_Library/Class/MSC/Src/usbh_msc.c \
STM32_USB_Host_Library/Class/MSC/Src/usbh_msc_scsi.c \
STM32_USB_Host_Library/Class/MSC/Src/usbh_msc_bot.c \
STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \

# C++ sources
CXX_SOURCES =  \
Src/main.cpp \
Src/SPI.cpp \
Src/RF24.cpp \

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC 	= $(GCC_PATH)/$(PREFIX)gcc
CXX	= $(GCC_PATH)/$(PREFIX)g++
AS 	= $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP 	= $(GCC_PATH)/$(PREFIX)objcopy
SZ 	= $(GCC_PATH)/$(PREFIX)size
else
CC 	= $(PREFIX)gcc
CXX	= $(PREFIX)g++
AS 	= $(PREFIX)gcc -x assembler-with-cpp
CP 	= $(PREFIX)objcopy
SZ 	= $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IHAL/Inc \
-IFatFs/src \
-ISTM32_USB_Host_Library/Core/Inc \
-ISTM32_USB_Host_Library/Class/MSC/Inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VGTx_FLASH.ld

# libraries
LIBS += -lc  -lnosys
LIBS += -lPDMFilter_CM4_GCC_wc32
LIBS += -lm
LIBDIR = -L.
LDFLAGS = $(MCU) -specs=nosys.specs -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of C++ program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
