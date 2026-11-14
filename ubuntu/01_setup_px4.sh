#!/usr/bin/env bash
# Step 1: get PX4 v1.17.0 and install its build tools. Run once. Asks for your sudo password.
#   bash ubuntu/01_setup_px4.sh
# Restart the computer afterwards (the PX4 setup adds you to the dialout group).
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR/.git" ]; then
	git clone --recursive --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
	echo "PX4 source already present in $PX4_DIR"
fi

# Installs the NuttX ARM toolchain (apt gcc-arm-none-eabi) and Gazebo Harmonic.
# Add --no-sim-tools to skip Gazebo.
bash "$PX4_DIR/Tools/setup/ubuntu.sh"

echo
echo "Done. Restart the computer, then run: bash ubuntu/02_build_px4_rplidar.sh"
