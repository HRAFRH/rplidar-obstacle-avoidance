#!/usr/bin/env bash
# Step 4: build ArduCopter "simulation on hardware" for the Pixhawk 6X.
# The real board runs a simulated quadcopter: IMU, GPS and compass are simulated on the board.
#   bash ubuntu/04_build_sim_on_hardware.sh            # build only
#   bash ubuntu/04_build_sim_on_hardware.sh --upload   # build and flash over USB
#
# READ BEFORE FLASHING
# - Props off. Motor outputs become live if SIM_OH_MASK is set.
# - This build has NO proximity support (HAL_PROXIMITY_ENABLED 0), so it cannot test the lidar
#   or obstacle avoidance. Use 05_sitl_rplidar.sh for that.
# - It shares parameter storage with normal firmware. Save the board's parameters first, and
#   after going back to normal ArduCopter reset them (set FORMAT_VERSION to 0 and reboot),
#   then reload ardupilot/rplidar_a2m12_telem2.param.
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BOARD="${BOARD:-Pixhawk6X}"

cd "$ARDUPILOT_DIR"
# --frame must be given together with --simclass.
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board "$BOARD" --vehicle copter \
	--simclass MultiCopter --frame quad "$@"

echo
echo "Firmware: $ARDUPILOT_DIR/build/$BOARD/bin/arducopter.apj"
