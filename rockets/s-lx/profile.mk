# Build glue for the S-LX guidance profile (UKROC 2027 competition).
# Included by src/Makefile.defs only when ROCKET=s-lx.

ROCKET_PROFILE_DIR = $(TOPDIR)/../rockets/s-lx
ROCKET_PROFILE_VPATH = $(ROCKET_PROFILE_DIR)/src:$(ROCKET_PROFILE_DIR)/include
ROCKET_PROFILE_CFLAGS = -I$(ROCKET_PROFILE_DIR)/include
