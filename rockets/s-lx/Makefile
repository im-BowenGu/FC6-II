# S-LX profile (UKROC 2027 competition): host tests and local CI.
#
# `make ci` is the local CI entry point. It never touches the upstream tree
# and does not require GitHub Actions.

CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
CPPFLAGS += -Iinclude
LDLIBS += -lm

BUILD := build
TARGET := $(BUILD)/test_slx_profile
OBJS := $(BUILD)/slx_profile.o $(BUILD)/test_slx_profile.o
SIM := sim/run_dynamic_sim.py
LIVE_CFD := cfd/run_live_cfd.py
MOTION := $(BUILD)/cfd/motion.json
# Override from the runner or the command line, e.g.
#   make cfd-record CFD_ARGS="--configs neutral,pitch_p15 --steps 2000"
CFD_ARGS ?=

.PHONY: all test ci ci-live host-test model-check sim-check verify cfd-record cfd-test clean

all: test

ci: test

test: host-test model-check sim-check

host-test: $(TARGET)
	./$(TARGET)

model-check:
	python3 tests/test_model_manifest.py

sim-check:
	python3 $(SIM) --check

verify: test

# Optional live-CFD leg of local CI. Requires an OpenCL device and FLUIDX3D_SRC;
# never invoked by GitHub Actions.
ci-live: cfd-test

# Run live FluidX3D over the selected fin states and record their forces as
# one motion lookup. No flow solver runs inside the 100 Hz flight loop.
cfd-record:
	python3 $(LIVE_CFD) --out $(BUILD)/cfd $(CFD_ARGS)

# Optional local CI gate on a machine with OpenCL: solve the fin states, then
# replay the recorded lookup through the real trajectory/firmware simulation.
cfd-test: host-test model-check cfd-record
	python3 $(SIM) --check --motion $(MOTION)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/slx_profile.o: src/slx_profile.c include/slx_profile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_slx_profile.o: tests/test_slx_profile.c include/slx_profile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDLIBS)

clean:
	rm -rf $(BUILD)
