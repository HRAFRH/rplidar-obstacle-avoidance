#!/usr/bin/env bash
# Copy the RPLidar driver into a PX4-Autopilot source tree and enable it for a board.
#
# Usage:  ./install_into_px4.sh <path to PX4-Autopilot> [board config]...
# Example: ./install_into_px4.sh ~/PX4-Autopilot boards/px4/fmu-v6x/default.px4board
#
# With no board given, it enables the driver for the Pixhawk 6X (px4_fmu-v6x_default).
set -euo pipefail

if [ $# -lt 1 ]; then
	echo "usage: $0 <path to PX4-Autopilot> [board config]..." >&2
	exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="$(cd "$1" && pwd)"
shift

if [ ! -f "$PX4/src/drivers/distance_sensor/Kconfig" ]; then
	echo "error: $PX4 does not look like a PX4-Autopilot tree" >&2
	exit 1
fi

BOARDS=("$@")

if [ ${#BOARDS[@]} -eq 0 ]; then
	BOARDS=("boards/px4/fmu-v6x/default.px4board")
fi

DEST="$PX4/src/drivers/distance_sensor/rplidar"
mkdir -p "$DEST"
cp "$HERE"/src/drivers/distance_sensor/rplidar/* "$DEST"/
echo "copied driver to $DEST"

# aid_switch: position source switch (GPS / GPS + optical flow / optical flow) from an RC switch
MDEST="$PX4/src/modules/aid_switch"
mkdir -p "$MDEST"
cp "$HERE"/src/modules/aid_switch/* "$MDEST"/
echo "copied aid_switch to $MDEST"

OPTION="CONFIG_DRIVERS_DISTANCE_SENSOR_RPLIDAR=y"
MOPTION="CONFIG_MODULES_AID_SWITCH=y"

for board in "${BOARDS[@]}"; do
	file="$PX4/$board"

	if [ ! -f "$file" ]; then
		echo "error: board config $file not found" >&2
		exit 1
	fi

	if grep -q "^$OPTION$" "$file"; then
		echo "already enabled in $board"

	else
		# keep the file sorted the way PX4 expects: insert after the other distance sensor line
		if grep -q "^CONFIG_COMMON_DISTANCE_SENSOR=y$" "$file"; then
			sed -i "/^CONFIG_COMMON_DISTANCE_SENSOR=y$/a $OPTION" "$file"

		else
			echo "$OPTION" >> "$file"
		fi

		echo "enabled in $board"
	fi

	if grep -q "^$MOPTION$" "$file"; then
		echo "aid_switch already enabled in $board"

	else
		if grep -q "^CONFIG_MODULES_AIRSPEED_SELECTOR=" "$file"; then
			sed -i "/^CONFIG_MODULES_AIRSPEED_SELECTOR=/i $MOPTION" "$file"

		else
			echo "$MOPTION" >> "$file"
		fi

		echo "aid_switch enabled in $board"
	fi
done

# PX4 patches (patches/*.patch), applied once:
#  - fmu-v6x-5v-rails-on-at-boot, cubeorangeplus-5v-rails-on-at-boot: keep the 5 V peripheral
#    rail on during boot. Without it, PX4 switches TELEM 5 V off and on early in boot, which can
#    hang a lidar powered from TELEM 5 V (seen on the 6X).
#  - rc-mc-apps-start-aid-switch: start aid_switch on multicopters (it does nothing while ASW_RC_CHAN is 0).
#  - mavlink-obstacle-distance-raw-fallback: send the raw obstacle_distance as OBSTACLE_DISTANCE
#    when Collision Prevention is not running, and stream it on USB, so QGC can show the scan.
# Set SKIP_BOARD_PATCHES=1 to leave PX4's own files alone.
if [ "${SKIP_BOARD_PATCHES:-0}" != "1" ]; then
	for patch in "$HERE"/patches/*.patch; do
		[ -f "$patch" ] || continue
		name="$(basename "$patch")"

		if git -C "$PX4" apply --reverse --check "$patch" 2>/dev/null; then
			echo "patch already applied: $name"

		elif git -C "$PX4" apply --check "$patch" 2>/dev/null; then
			git -C "$PX4" apply "$patch"
			echo "applied patch: $name"

		else
			echo "error: patch $name does not apply to this PX4 tree" >&2
			exit 1
		fi
	done
fi
