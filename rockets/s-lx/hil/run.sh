#!/bin/sh
# Start the C++ bridge (Verilated RTL + real C guidance core) and the physics
# plant together, then stop the bridge when the plant reports completion.
set -eu
cd "$(dirname "$0")"
mkdir -p logs
# Stale DONE flags from an interrupted run would make the bridge exit before
# the new plant publishes its first frame.
rm -f /dev/shm/slx_hil_sensors /dev/shm/slx_hil_fins
./bin/slx-hil-bridge --out logs/bridge.csv --max-ms 45000 2>logs/bridge.stderr &
bridge_pid=$!
trap 'kill "$bridge_pid" 2>/dev/null || true' EXIT INT TERM
set +e
PYTHONPATH="plant${PYTHONPATH:+:$PYTHONPATH}" python3 plant/plant.py "$@"
status=$?
set -e
# Give the bridge time to consume the DONE frame and flush its telemetry.
i=0
while kill -0 "$bridge_pid" 2>/dev/null && [ "$i" -lt 100 ]; do
    sleep 0.1
    i=$((i + 1))
done
kill "$bridge_pid" 2>/dev/null || true
wait "$bridge_pid" 2>/dev/null || true
trap - EXIT INT TERM
exit "$status"
