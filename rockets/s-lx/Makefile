CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
CPPFLAGS += -Iinclude
LDLIBS += -lm
PYTHON ?= python3
TAICHI_PYTHON ?= $(PYTHON)
TAICHI_LD_LIBRARY_PATH ?=
TAICHI_RUN = env PYTHONPATH="$(abspath ../..):$$PYTHONPATH" LD_LIBRARY_PATH="$(TAICHI_LD_LIBRARY_PATH)" "$(TAICHI_PYTHON)"
CARGO ?= cargo

BUILD := build
TARGETS := $(BUILD)/test_slx_profile $(BUILD)/test_slx_aero
OBJS := $(BUILD)/slx_profile.o $(BUILD)/slx_aero.o \
        $(BUILD)/test_slx_profile.o $(BUILD)/test_slx_aero.o
SIM := sim/run_dynamic_sim.py
LIVE_CFD := cfd/run_live_cfd.py
CFD_CONFIG := config/cfd.json
VIS_CONFIG := config/vis.json
VIS_DIR := ../../shared/vis/rust
MOTION := $(BUILD)/cfd/motion.json
CFD_ARGS ?=
CFD_LIVE_ARGS ?=
VIS_ARGS ?=

VIS_BIN := $(VIS_DIR)/target/release/rocket-vis

.PHONY: all test ci ci-live host-test model-check sim-check verify cfd-record cfd-test cfd-fields cfd-live vis-rust vis-rust-run clean

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

ci-live: cfd-test

cfd-record:
	@$(TAICHI_RUN) -c 'import taichi, numpy' 2>/dev/null || { echo "Taichi/numpy unavailable in $(TAICHI_PYTHON); override TAICHI_PYTHON=PATH"; exit 1; }
	$(TAICHI_RUN) $(LIVE_CFD) --config $(CFD_CONFIG) --out $(BUILD)/cfd $(CFD_ARGS)

cfd-test: host-test model-check cfd-record
	$(PYTHON) $(SIM) --check --motion $(MOTION)

cfd-fields:
	@$(TAICHI_RUN) -c 'import taichi, numpy' 2>/dev/null || { echo "Taichi/numpy unavailable in $(TAICHI_PYTHON); override TAICHI_PYTHON=PATH"; exit 1; }
	$(TAICHI_RUN) $(LIVE_CFD) --config $(CFD_CONFIG) --out $(BUILD)/cfd --dump-field $(CFD_ARGS)

cfd-live:
	@$(TAICHI_RUN) -c 'import taichi, numpy' 2>/dev/null || { echo "Taichi/numpy unavailable in $(TAICHI_PYTHON); override TAICHI_PYTHON=PATH"; exit 1; }
	$(TAICHI_RUN) -m shared.cfd.live_server --config $(CFD_CONFIG) --socket $(BUILD)/cfd/field.sock --log hil/logs/bridge.csv $(CFD_LIVE_ARGS)

vis-rust:
	@command -v $(CARGO) >/dev/null || { echo "cargo not found; install the Rust toolchain"; exit 1; }
	$(CARGO) build --release --offline --manifest-path $(VIS_DIR)/Cargo.toml

vis-rust-run: vis-rust
	DISPLAY=$${DISPLAY:-:0} $(VIS_BIN) --config $(VIS_CONFIG) $(VIS_ARGS)

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
