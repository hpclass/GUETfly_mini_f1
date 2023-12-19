# Makefile for multiwii project

# Toolchain
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy

# Compiler flags
CFLAGS = -mcpu=cortex-m3 \
			-mthumb -std=gnu11 -Wall \
			-DSTM32F10X_MD \
			-DGUET_FLY_MINI_V1
ASFLAGS = -mcpu=cortex-m3 -mthumb

# Directories
SRC_DIR = ./multiwii_2.4
USER_DIR = ./USER 
CMSIS_DIR = ./Libraries/CMSIS/CM3/CoreSupport

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.c) $(USER_DIR)/stm32f10x_it.c $(USER_DIR)/timer.c
ASM_FILES = $(wildcard $(SRC_DIR)/*.s*)
CMSIS_FILES = $(CMSIS_DIR)/core_cm3.c

# Header files
INC_DIRS = -I$(SRC_DIR) -I$(USER_DIR) -I$(CMSIS_DIR) \
-I./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/ \
-I./Libraries/STM32F10x_StdPeriph_Driver/inc/ \
-I./USB

SOURCES = \
    ./multiwii_2.4/Alarms.c \
    ./multiwii_2.4/EEPROM.c \
    ./multiwii_2.4/GPS.c \
    ./multiwii_2.4/IMU.c \
    ./multiwii_2.4/Protocol.c \
    ./multiwii_2.4/MultiWii.c \
    ./multiwii_2.4/Output.c \
    ./multiwii_2.4/Sensors.c \
    ./multiwii_2.4/Serial.c \
    ./multiwii_2.4/RX.c \
    ./USER/stm32f10x_it.c \
    ./USER/timer.c \
    ./USER/AltHold.c \
    ./USER/delay.c \
    ./USER/re_eeprom.c \
    ./USER/sys.c \
    ./USER/gy_86.c \
    ./USER/SPL06_001.c \
    ./Libraries/CMSIS/CM3/CoreSupport/core_cm3.c \
	./USER/system_stm32f10x.c \
	./Libraries/STM32F10x_StdPeriph_Driver/src/misc.c \
	./USB/usart.c \
	./USB/USB_CH341.c \
	
#	./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/arm/startup_stm32f10x_hd.s \
# Object files
OBJECTS = $(SOURCES:.c=.o)

# Output file
TARGET = multiwii_2.4_STM32.elf

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(INC_DIRS) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(TARGET)
