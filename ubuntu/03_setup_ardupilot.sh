#!/usr/bin/env bash
# Step 3: get the ArduPilot source and install its build tools. Run once. Asks for your sudo password.
#   bash ubuntu/03_setup_ardupilot.sh
# Log out and back in afterwards so the new PATH from ~/.profile is active.
#
# Note: this adds ArduPilot's ARM compiler (GCC 10, in /opt) to the front of PATH in ~/.profile.
# 02_build_px4_rplidar.sh removes it again for PX4 builds, so both can live on one machine.
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
else
	echo "ArduPilot source already present in $ARDUPILOT_DIR"
fi

cd "$ARDUPILOT_DIR"
Tools/environment_install/install-prereqs-ubuntu.sh -y

echo
echo "Done. Log out and back in, then run 04 (simulation on hardware) or 05 (SITL)."
