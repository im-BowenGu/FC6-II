#!/bin/sh
# S-LX local CI entry point. Runs entirely on the developer's machine / local
# CI server -- nothing here depends on GitHub Actions, a PR, or upstream refs.
#
#   ./rockets/s-lx/ci/run-local.sh          # offline: host tests + frozen-lookup sim
#   RUN_LIVE_CFD=1 ./rockets/s-lx/ci/run-local.sh   # also solve fin states with FluidX3D
#
# Simulation telemetry and events are written to rockets/s-lx/sim/logs/ on
# every run, including failures (override with LOG_DIR=/path/to/logs).
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
cd "$root"

echo "== S-LX local CI: host tests, model manifest, 100 Hz simulation =="
make -C rockets/s-lx ci

if [ "${RUN_LIVE_CFD:-0}" = "1" ]; then
    echo "== S-LX local CI: live FluidX3D fin-state sweep + replay =="
    make -C rockets/s-lx ci-live
fi

echo "S-LX local CI PASSED"
