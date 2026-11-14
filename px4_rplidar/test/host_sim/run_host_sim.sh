#!/usr/bin/env bash
# Build and run the host test bench: the real rplidar.cpp against a simulated RPLidar.
#   bash run_host_sim.sh <path to PX4-Autopilot>
# Uses g++ by default. On Windows without a compiler: pip install ziglang, then
#   CXX="python -m ziglang c++" bash run_host_sim.sh <path to PX4-Autopilot>
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
DRIVER="$HERE/../../src/drivers/distance_sensor/rplidar"
OUT="${OUT:-/tmp/rplidar_host_sim}"
CXX="${CXX:-g++}"

# The language mode and warning flags are the ones PX4 builds with (cmake/px4_add_common_flags.cmake).
$CXX -std=gnu++14 -D_USE_MATH_DEFINES -O1 \
	-Wall -Wextra -Werror -Wshadow -Wdouble-promotion -Wfloat-equal -Wno-unused-parameter \
	-I"$HERE/stubs" -I"$DRIVER" \
	-I"$PX4/src/lib" -I"$PX4/src/lib/matrix" -I"$PX4/src/lib/collision_prevention" -I"$PX4/src" \
	"$HERE/host_sim.cpp" "$DRIVER/rplidar.cpp" "$PX4/src/lib/collision_prevention/ObstacleMath.cpp" \
	-o "$OUT"

"$OUT"
