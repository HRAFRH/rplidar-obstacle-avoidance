#!/usr/bin/env bash
# Step 5: ArduCopter SITL with a simulated RPLidar A2, for testing obstacle avoidance on the PC.
#   bash ubuntu/05_sitl_rplidar.sh
#
# The simulated lidar sees a fixed grid of posts (1 m radius, every 10 m) around
# 51.8752066 N, 14.6487830 E. The start location below is therefore required: anywhere else
# the lidar sees nothing.
#
# In the MAVProxy console that opens:
#   script /tmp/post-locations.scr     draw the posts on the map
#   mode loiter
#   arm throttle
#   rc 3 1600                          climb, then: rc 3 1500 to hold height
#   rc 2 1450                          fly slowly forward toward a post; it should stop about 2 m away
# Mission Planner or QGroundControl can also connect (UDP 14550) to show the proximity radar.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

cd "$ARDUPILOT_DIR"
./Tools/autotest/sim_vehicle.py -v ArduCopter \
	-A "--serial5=sim:rplidara2" \
	-l 51.8752066,14.6487840,54.15,0 \
	--add-param-file="$HERE/sitl_rplidar.parm" \
	--map --console "$@"
