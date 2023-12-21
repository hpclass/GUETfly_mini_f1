# Makefile for multiwii project

# Toolchain
CC = ${CROSS_COMPILE}gcc
AS = ${CROSS_COMPILE}as
LD = ${CROSS_COMPILE}ld
OBJCOPY = ${CROSS_COMPILE}objcopy

# Compiler flags
CFLAGS = -mcpu=cortex-m3 \
         -mthumb -std=gnu11 -Wall \
         -DSTM32F10X_MD \
         -DGUET_FLY_MINI_V1  \
		 -fno-stack-protector \
		 -mfloat-abi=soft 
ASFLAGS = -mcpu=cortex-m3 -mthumb

LDFLAGS = -lm
# Directories
SRC_DIR = ./multiwii_2.4
USER_DIR = ./USER 
CMSIS_DIR = ./Libraries/CMSIS/CM3/CoreSupport

# Source files
ASM_FILES = ./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/arm/startup_stm32f10x_hd.S 
CMSIS_FILES = $(CMSIS_DIR)/core_cm3.c

# Header files
INC_DIRS = -I$(SRC_DIR) -I$(USER_DIR) -I$(CMSIS_DIR) \
           -I./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/ \
           -I./Libraries/STM32F10x_StdPeriph_Driver/inc/ \
           -I./USB

SOURCES = \
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
    ./multiwii_2.4/init_c.c \
    ./USER/AltHold.c \
    ./USER/SPL06_001.c \
    ./USER/main.c \
    ./USER/re_eeprom.c \
    ./USER/system_stm32f10x.c \
    ./USER/Ano_OF.c \
    ./USER/math_.c \
    ./USER/soft_iic.c \
    ./USER/test1.c \
    ./USER/DJI_Guidance_usat.c \
    ./USER/delay.c \
    ./USER/oled.c \
    ./USER/spi.c \
    ./USER/timer.c \
    ./USER/IIC_SOFTWARE.c \
    ./USER/gy_86.c \
    ./USER/optic_.c \
    ./USER/stm32f10x_it.c \
    ./USER/INS.c \
    ./USER/ov7670_Dir.c \
    ./USER/sys.c \
    ./USB/usart.c \
    ./USB/USB_CH341.c \
    ./Libraries/CMSIS/CM3/CoreSupport/core_cm3.c \
    ./USER/system_stm32f10x.c \
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

# 在OBJECTS的定义中，使用grep过滤大小写不同的.s文件
OBJECTS = $(filter-out $(shell grep -il "\.s" $(SOURCES)), $(SOURCES:.c=.o))

TARGET = multiwii_2.4_STM32.elf

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $^ 

%.o: %.c
	$(CC) $(INC_DIRS) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(AS) $(ASFLAGS) -o $@ $<

clean:
	rm -f $(filter-out $(ASM_FILES), $(OBJECTS) $(TARGET))
