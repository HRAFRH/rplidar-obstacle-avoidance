#!/usr/bin/env bash
# Step 2: test the RPLidar protocol code, install the driver into PX4 and build the firmware.
#   bash ubuntu/02_build_px4_rplidar.sh            # build for the Pixhawk 6X
#   bash ubuntu/02_build_px4_rplidar.sh upload     # build and flash over USB (board plugged in)
# Other board: BOARD=px4_fmu-v6c_default BOARD_CONFIG=boards/px4/fmu-v6c/default.px4board bash ubuntu/02_build_px4_rplidar.sh
#
# The Pixhawk 6X target is the "multicopter" variant. px4_fmu-v6x_default does not fit in flash
# with ARM GCC 10.3 (Ubuntu 22.04): it overflowed by 20.9 KB, of which the driver is 4.4 KB.
# The variant drops the fixed-wing and VTOL modules and inherits default.px4board, where the
# driver is enabled.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BOARD="${BOARD:-px4_fmu-v6x_multicopter}"
BOARD_CONFIG="${BOARD_CONFIG:-boards/px4/fmu-v6x/default.px4board}"

# ArduPilot's setup puts its own, older ARM compiler first in PATH. PX4 should use the apt one.
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/gcc-arm-none-eabi' | paste -sd ':')"
export PATH
echo "ARM compiler: $(command -v arm-none-eabi-gcc) ($(arm-none-eabi-gcc -dumpversion))"

echo "--- host test of the protocol decoding ---"
g++ -std=c++14 -Wall -Wextra -I"$ROOT/px4_rplidar/src/drivers/distance_sensor/rplidar" \
	"$ROOT/px4_rplidar/test/test_protocol.cpp" -o /tmp/test_rplidar_protocol
/tmp/test_rplidar_protocol

echo "--- host test bench: the real driver against a simulated lidar ---"
bash "$ROOT/px4_rplidar/test/host_sim/run_host_sim.sh" "$PX4_DIR" | grep -E "^== |FAIL|PASSED|FAILED"

echo "--- install the driver into $PX4_DIR ---"
bash "$ROOT/px4_rplidar/install_into_px4.sh" "$PX4_DIR" "$BOARD_CONFIG"

echo "--- build $BOARD ---"
cd "$PX4_DIR"
make "$BOARD" ${1:-}

echo
echo "Firmware: $PX4_DIR/build/$BOARD/$BOARD.px4"
