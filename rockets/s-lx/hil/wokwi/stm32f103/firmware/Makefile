# Minimal Wokwi STM32F103 harness firmware. Links the real S-LX guidance core;
# the sensor/servo layer is scenario-driven (Wokwi has no BMP280/ICM20689/
# PCA9685 parts). Requires arm-none-eabi-gcc (e.g. nix-shell -p gcc-arm-embedded).
ARM_GCC ?= arm-none-eabi-gcc
ROOT := $(abspath ../../../../../..)
PROFILE := $(ROOT)/rockets/s-lx
CC := $(ARM_GCC)

CFLAGS := -mcpu=cortex-m3 -mthumb -Os -std=c11 -Wall -Wextra \
          -ffunction-sections -fdata-sections \
          -I$(PROFILE)/include \
          -DSLX_WOKWI_HARNESS
LDFLAGS := -mcpu=cortex-m3 -mthumb -nostartfiles \
           -Tlinker.ld -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs

.PHONY: all clean
all: firmware.elf

firmware.elf: main.c $(PROFILE)/src/slx_profile.c $(PROFILE)/src/slx_aero.c linker.ld
	$(CC) $(CFLAGS) -c main.c -o main.o
	$(CC) $(CFLAGS) -c $(PROFILE)/src/slx_profile.c -o slx_profile.o
	$(CC) $(CFLAGS) -c $(PROFILE)/src/slx_aero.c -o slx_aero.o
	$(CC) $(LDFLAGS) main.o slx_profile.o slx_aero.o -lm -o $@

clean:
	rm -f firmware.elf main.o slx_profile.o slx_aero.o
