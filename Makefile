# Makefile for multiwii project
PROJECT := multiwii
# Toolchain
CC = ${CROSS_COMPILE}gcc
AS = ${CROSS_COMPILE}as
LD = ${CROSS_COMPILE}ld
CC        := $(CROSS_COMPILE)gcc
CXX       := $(CROSS_COMPILE)g++
CP        := $(CROSS_COMPILE)objcopy
GDB       := $(CROSS_COMPILE)gdb
SIZE      := $(CROSS_COMPILE)size
AS        := $(CC) -x assembler-with-cpp
HEX       := $(CP) -O ihex
BIN       := $(CP) -O binary -S
OBJCOPY = ${CROSS_COMPILE}objcopy

# Compiler flags

DEFS  := $(DDEFS) -DRUN_FROM_FLASH=1 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER 



OPT   += -Os
OPT   += -fsingle-precision-constant
OPT   += -fno-common
OPT   += -ffunction-sections
OPT   += -fdata-sections

SPECS := --specs=rdimon.specs -u _printf_float

# Directories
SRC_DIR = ./multiwii_2.4
USER_DIR = ./USER 
CMSIS_DIR = ./Libraries/CMSIS/CM3/CoreSupport



# Header files
INC_STM32_LIB := -I./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/ \
           -I./Libraries/STM32F10x_StdPeriph_Driver/inc/ \
		   -I./USB \
		   -I./LL_Drievers/STM32/inc \

INC_GD32_LIB := -I./Libraries/GD32F3x0_standard_peripheral/Include/ \
				-I./Libraries/CMSIS/GD32F3x0/Include/ \
				-I./Libraries/CMSIS/ \
				-I./LL_Drievers/GD32/inc \
				
INC_DIRS := -I$(SRC_DIR) -I$(USER_DIR) -I$(CMSIS_DIR) \


MultiWii_SOURCES = \
    ./multiwii_2.4/Alarms.c \
    ./multiwii_2.4/IMU.c \
    ./multiwii_2.4/Output.c \
    ./multiwii_2.4/Sensors.c \
    ./multiwii_2.4/EEPROM.c \
    ./multiwii_2.4/LCD.c \
    ./multiwii_2.4/Protocol.c \
    ./multiwii_2.4/Serial.c \
    ./multiwii_2.4/GPS.c \
    ./multiwii_2.4/MultiWii.c \
    ./multiwii_2.4/RX.c \

HAL_SOURCES = \
    ./USER/AltHold.c \
    ./USER/SPL06_001.c \
    ./USER/re_eeprom.c \
    ./USER/Ano_OF.c \
    ./USER/math_.c \
    ./USER/DJI_Guidance_usat.c \
    ./USER/oled.c \
    ./USER/optic_.c \
    ./USER/INS.c \
    ./USER/ov7670_Dir.c \

SOURCES_STM32_LIBS := \
    ./USB/USB_CH341.c \
    ./LL_Drievers/STM32/src/system_stm32f10x.c \
	./LL_Drievers/STM32/src/delay.c \
	./LL_Drievers/STM32/src/sys.c \
	./LL_Drievers/STM32/src/timer.c \
	./LL_Drievers/STM32/src/soft_iic.c \
	./LL_Drievers/STM32/src/IIC_SOFTWARE.c \
	./LL_Drievers/STM32/src/spi.c \
	./LL_Drievers/STM32/src/usart2.c \
	./LL_Drievers/STM32/src/gy_86.c \
	./LL_Drievers/STM32/src/stm32f10x_it.c \
    ./Libraries/CMSIS/CM3/CoreSupport/core_cm3.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/misc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c \
    ./Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c \

SOURCES_GD32_LIBS := \
	./Libraries/CMSIS/GD32F3x0/Source/system_gd32f3x0.c \
	./LL_Drievers/GD32/src/i2c.c \
	./LL_Drievers/GD32/src/spi.c   \
	./LL_Drievers/GD32/src/uart.c \
	./LL_Drievers/GD32/src/leds.c  \
	./LL_Drievers/GD32/src/timer.c \
	./LL_Drievers/GD32/src/sys.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_adc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_cec.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_cmp.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_crc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_ctc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_dac.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_dbg.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_dma.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_exti.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_fmc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_fwdgt.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_gpio.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_i2c.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_misc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_pmu.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_rcu.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_rtc.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_spi.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_syscfg.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_timer.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_tsi.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_usart.c \
	./Libraries/GD32F3x0_standard_peripheral/Source/gd32f3x0_wwdgt.c \

# 在OBJECTS的定义中，使用grep过滤大小写不同的.s文件
ifeq ($(MAKECMDGOALS),GD32)
# Source files
	ASM_FILES = ./Libraries/CMSIS/GD32F3x0/Source/GNU/startup_gd32f3x0.s
	SOURCES := $(SOURCES_GD32_LIBS) $(HAL_SOURCES) $(MultiWii_SOURCES) $(USB_SOURCES)
	TARGET := $(PROJECT)_GD32
	INC_DIRS += $(INC_GD32_LIB)
	DEFS += -DGD32F330
	MCU   := cortex-m4
	LINK_SCRIPT := gd32f3x0_flash.lds
else
# Source files
	ASM_FILES = ./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_hd.s
	CMSIS_FILES = $(CMSIS_DIR)/core_cm3.c
	SOURCES := $(SOURCES_STM32_LIBS) $(HAL_SOURCES) $(MultiWii_SOURCES) $(USB_SOURCES)
	TARGET := $(PROJECT)_STM32
	INC_DIRS += $(INC_STM32_LIB)
	DEFS += -DSTM32F10X_MD
	MCU   := cortex-m3
	LINK_SCRIPT := stm32f10x_flash.lds
endif



FLAGS_MCU := -mcpu=$(MCU)
FLAGS_AS  := $(SPECS) $(FLAGS_MCU) $(OPT) -c -g -gdwarf-2 -mthumb
FLAGS_C   := $(SPECS) $(FLAGS_MCU) $(OPT) -c -g -gdwarf-2 -mthumb \
             -fomit-frame-pointer -Wall -fverbose-asm $(DEFS)
FLAGS_CXX := $(SPECS) $(FLAGS_MCU) $(OPT) -c -g -gdwarf-2 -mthumb \
             -fomit-frame-pointer -Wall -fverbose-asm -fno-exceptions \
             -fno-rtti -fno-threadsafe-statics -fvisibility=hidden -std=c++11 \
             $(DEFS)
FLAGS_LD  := $(SPECS) $(FLAGS_MCU) $(OPT) -lm -g -gdwarf-2 -mthumb \
             -nostartfiles -Xlinker --gc-sections -T$(LINK_SCRIPT) \
             -Wl,-Map=$(TARGET).map,--cref,--no-warn-mismatch

			 
OBJECTS    := $(filter %.o, $(ASM_FILES:.s=.o)) $(filter %.o, $(SOURCES:.c=.o))

.PHONY: all GD32 STM32 clean help

all: $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin
	$(SIZE) $(PROJECT).elf -A

GD32: $(TARGET).elf $(TARGET).hex $(TARGET).bin
	$(SIZE) $(TARGET).elf -A

STM32:$(TARGET).elf $(TARGET).hex $(TARGET).bin
	$(SIZE) $(TARGET).elf -A

$(TARGET).elf: $(OBJECTS) 
	$(CC) $(OBJECTS) $(FLAGS_LD) -o $@

%.o: %.c
	$(CC) $(INC_DIRS) $(FLAGS_C) -c -o $@ $<

%.o: %.s
	$(AS) $(FLAGS_AS) $< -o $@
%.hex: %.elf
	$(HEX) $< $@

%.bin: %.elf
	$(BIN) $< $@

clean:
	rm -f $(filter-out $(ASM_FILES), $(OBJECTS) $(TARGET))

help: 
	echo "usage : make "
