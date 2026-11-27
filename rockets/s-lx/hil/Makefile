# S-LX hardware-in-the-loop harness.
#
# Local only: no GitHub Actions, no Wokwi token required for the offline path.
#   make -C rockets/s-lx/hil build     # Verilate RTL + build C++ bridge
#   make -C rockets/s-lx/hil run       # plant + real guidance core over shm
#   make -C rockets/s-lx/hil vis       # browser GUI at http://127.0.0.1:8765
#   make -C rockets/s-lx/hil wokwi-lint    # lint Wokwi projects (needs Wokwi CLI)
#   make -C rockets/s-lx/hil wokwi-stm32   # run STM32 Wokwi smoke (needs token)

ROOT := $(abspath ../../..)
HIL := $(abspath .)
OBJ := $(HIL)/obj_dir
OBJ_LOCAL := $(HIL)/obj
BIN := $(HIL)/bin
PROFILE_INC := $(ROOT)/rockets/s-lx/include
PROFILE_SRC := $(ROOT)/rockets/s-lx/src/slx_profile.c $(ROOT)/rockets/s-lx/src/slx_aero.c
VERILATOR ?= verilator
CC ?= gcc
CXX ?= g++
# Local Wokwi tooling. The token file is outside the repo and MUST NOT be
# committed; override these variables on the command line in other setups.
WOKWI_CLI ?= /tmp/opencode/wokwi-tools/wokwi-cli
WOKWI_TOKEN_ENV ?= /tmp/opencode/wokwi-token.env
VBIN := $(shell command -v $(VERILATOR) 2>/dev/null || echo verilator)
VROOT := $(shell dirname $(shell dirname $(VBIN)))
VINC := $(VROOT)/share/verilator/include
VCPP := $(wildcard $(OBJ)/Vslx_periph*.cpp)

.PHONY: all build rtl bridge run run-mujoco vis wokwi-lint wokwi-stm32 clean logs

all: build

# We deliberately avoid Verilator's --build driver because it shells out to
# GNU make, which is not guaranteed on PATH in every local CI shell. Instead we
# generate the Verilated C++ and link it directly with $(CXX).
rtl:
	$(VERILATOR) --cc --top-module slx_periph --Mdir $(OBJ) -Wno-fatal \
		-I$(HIL)/include rtl/slx_periph.sv

$(OBJ_LOCAL)/%.o: $(ROOT)/rockets/s-lx/src/%.c $(PROFILE_INC)/%.h
	mkdir -p $(OBJ_LOCAL)
	$(CC) -std=c11 -O2 -Wall -Wextra -I$(PROFILE_INC) -c $< -o $@

PROFILE_OBJS := $(addprefix $(OBJ_LOCAL)/,slx_profile.o slx_aero.o)

bridge: rtl $(PROFILE_OBJS)
	mkdir -p $(BIN) $(HIL)/logs
	$(CXX) -std=c++17 -O2 -Wall -Wno-sign-compare -Wno-maybe-uninitialized \
		-I$(HIL)/include -I$(PROFILE_INC) -I$(OBJ) -I$(VINC) \
		cpp/bridge.cpp $(PROFILE_OBJS) \
		$(VCPP) $(VINC)/verilated.cpp $(VINC)/verilated_threads.cpp \
		-lm -o $(BIN)/slx-hil-bridge

build: bridge

run: build
	./run.sh --backend cfd

run-mujoco: build
	./run.sh --backend mujoco

vis:
	python3 vis/server.py

wokwi-lint:
	$(WOKWI_CLI) lint wokwi/esp32-c3
	$(WOKWI_CLI) lint wokwi/stm32f103

# Runnable Wokwi smoke test: the STM32 harness links the unchanged C guidance
# core, substitutes scenario-driven sensor values for the parts Wokwi lacks,
# and streams the real CSV telemetry format over USART1 (PA9 -> serial monitor).
wokwi-stm32:
	$(MAKE) -C wokwi/stm32f103/firmware
	set -a; . $(WOKWI_TOKEN_ENV); set +a; \
	  $(WOKWI_CLI) --timeout 40000 --scenario scenario.yaml \
	  --expect-text "S-LX HIL WOKWI DONE" wokwi/stm32f103

logs:
	mkdir -p $(HIL)/logs

clean:
	rm -rf $(OBJ) $(OBJ_LOCAL) $(BIN) $(HIL)/logs/*
