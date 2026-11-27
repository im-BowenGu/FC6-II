#
# S-LX STM32F103C8T6 flight target (UKROC 2027 competition)
#

include ../stm32f1/Makefile.defs

INC = \
	ao.h \
	ao_arch.h \
	ao_arch_funcs.h \
	ao_pins.h \
	ao_product.h \
	ao_adc_single.h \
	ao_bmp280.h \
	ao_icm20689.h \
	ao_pca9685.h \
	ao_fat.h \
	slx_profile.h \
	stm32f1.h \
	Makefile

#
# Minimal AltOS kernel + drivers for autonomous flight. No USB, serial,
# command shell, radio, GPS or pyro sources are linked.
#
ALTOS_SRC = \
	ao_clock.c \
	ao_timer.c \
	ao_mutex.c \
	ao_boot_chain.c \
	ao_interrupt.c \
	ao_product.c \
	ao_romconfig.c \
	ao_task.c \
	ao_panic.c \
	ao_dma_stm.c \
	ao_spi_stm.c \
	ao_i2c_stm.c \
	ao_led.c \
	ao_adc_single_stm.c \
	ao_sdcard.c \
	ao_bufio.c \
	ao_fat.c \
	ao_bmp280.c \
	ao_icm20689.c \
	ao_pca9685.c \
	slx_profile.c

PRODUCT=S-LX-F103
PRODUCT_DEF=-DSLX_F103 -DFAT_COMMANDS=0
IDPRODUCT=0x0029

CFLAGS = $(PRODUCT_DEF) $(STM32F1_CFLAGS)

PROGNAME=s-lx-bluepill-f103
PROG=$(PROGNAME)-$(VERSION).elf
HEX=$(PROGNAME)-$(VERSION).ihx
FLASH_PROG=flash-loader/$(PROGNAME)-altos-flash-$(VERSION).elf
MAKEBIN=$(TOPDIR)/../ao-tools/ao-makebin/ao-makebin

SRC=$(ALTOS_SRC) ao_slx.c
OBJ=$(SRC:.c=.o)

# The inherited AltOS FAT/SD card drivers predate the strict -Wconversion and
# -Wformat checks enabled for this fork. Modern GCC (15.x) reports a large
# number of benign narrowing conversions in that legacy code. Keep the flight
# target buildable without changing shared driver semantics.
ao_fat.o: CFLAGS += -Wno-conversion -Wno-sign-conversion -Wno-format
ao_sdcard.o: CFLAGS += -Wno-conversion -Wno-sign-conversion -Wno-format

all: $(PROG) $(HEX) $(FLASH_PROG)

$(PROG): Makefile $(OBJ) altos.ld
	$(call quiet,CC) $(LDFLAGS) -o $(PROG) $(OBJ) $(LIBS)

$(HEX): $(PROG)
	$(call quiet,ELFTOHEX) --output=$@ $*.elf

$(FLASH_PROG): FRC
	+cd flash-loader && $(MAKE)

$(OBJ): $(INC)

FRC:

# Optional combined DFU/HEX image; requires ao-tools to have been built.
combined: $(PROG) $(FLASH_PROG)
	$(MAKEBIN) --dfu --output=$(PROGNAME)-combined-$(VERSION).dfu \
		--base=0x08000000 $(FLASH_PROG) $(PROG)
	$(ELFTOHEX) --nosym --output=$(PROGNAME)-combined-$(VERSION).ihx \
		$(FLASH_PROG) $(PROG)

distclean: clean

clean:
	rm -f *.o $(PROGNAME)-*.elf $(PROGNAME)-*.ihx $(PROGNAME)-*.map
	rm -f ao_product.h
	$(MAKE) -C flash-loader clean

install:

uninstall:
