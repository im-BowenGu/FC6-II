# Build glue for the UKROC 2027 guidance profile.
# Included by src/Makefile.defs only when ROCKET=ukroc-2027.

ROCKET_PROFILE_DIR = $(TOPDIR)/../rockets/ukroc-2027
ROCKET_PROFILE_VPATH = $(ROCKET_PROFILE_DIR)/src:$(ROCKET_PROFILE_DIR)/include
ROCKET_PROFILE_CFLAGS = -I$(ROCKET_PROFILE_DIR)/include
