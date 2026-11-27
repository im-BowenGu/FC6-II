# S-LX profile (UKROC 2027 competition): host tests and local CI.
#
# `make ci` is the local CI entry point. It never touches the upstream tree
# and does not require GitHub Actions.

CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
CPPFLAGS += -Iinclude
LDLIBS += -lm
PYTHON ?= python3

# Taichi wheels are built for Python 3.11, not every host default. Override
# both variables when the venv lives elsewhere; the defaults match the local
# CI sandbox (CUDA driver + zlib + libstdc++ + libX11 are needed by Taichi's
# native extension).
TAICHI_PYTHON ?= /tmp/opencode/taichi-venv/bin/python
TAICHI_LD_LIBRARY_PATH ?= /nix/store/hi6r6y54znvyhmj554k0j59i281ahlrj-nvidia-x11-580.196.07/lib:/nix/store/kp68n1q6r8lzpf5mxjmfasn3zq2pql4x-zlib-1.3.2/lib:/nix/store/myhwfdgqm39yms5clinafii9i1nmq43h-gcc-16.2.0-lib/lib:/nix/store/nz6yl26svfj0qap9qnpvd26yp2gxn3xl-libx11-1.8.14/lib
TAICHI_RUN = env LD_LIBRARY_PATH="$(TAICHI_LD_LIBRARY_PATH)" "$(TAICHI_PYTHON)"

BUILD := build
TARGETS := $(BUILD)/test_slx_profile $(BUILD)/test_slx_aero
OBJS := $(BUILD)/slx_profile.o $(BUILD)/slx_aero.o \
        $(BUILD)/test_slx_profile.o $(BUILD)/test_slx_aero.o
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

host-test: $(TARGETS)
	./$(BUILD)/test_slx_profile
	./$(BUILD)/test_slx_aero

model-check:
	$(PYTHON) tests/test_model_manifest.py

sim-check:
	$(PYTHON) $(SIM) --check

verify: test

# Optional live-CFD leg of local CI. Requires a CUDA-capable GPU and Taichi;
# never invoked by GitHub Actions.
ci-live: cfd-test

# Run the Taichi D3Q19 LBM over the selected fin states on CUDA and record
# their calibrated forces as one motion lookup. No flow solver runs inside the
# 100 Hz flight loop.
cfd-record:
	@test -x "$(TAICHI_PYTHON)" || { echo "Taichi Python not found: $(TAICHI_PYTHON) (override TAICHI_PYTHON=)"; exit 1; }
	$(TAICHI_RUN) $(LIVE_CFD) --out $(BUILD)/cfd $(CFD_ARGS)

# Optional local CI gate on a machine with CUDA: solve the fin states, then
# replay the recorded lookup through the real trajectory/firmware simulation.
cfd-test: host-test model-check cfd-record
	$(PYTHON) $(SIM) --check --motion $(MOTION)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/slx_profile.o: src/slx_profile.c include/slx_profile.h include/slx_aero.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/slx_aero.o: src/slx_aero.c include/slx_aero.h include/slx_profile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_slx_profile.o: tests/test_slx_profile.c include/slx_profile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_slx_aero.o: tests/test_slx_aero.c include/slx_aero.h include/slx_profile.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_slx_profile: $(BUILD)/slx_profile.o $(BUILD)/slx_aero.o $(BUILD)/test_slx_profile.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/test_slx_aero: $(BUILD)/slx_profile.o $(BUILD)/slx_aero.o $(BUILD)/test_slx_aero.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

clean:
	rm -rf $(BUILD)
