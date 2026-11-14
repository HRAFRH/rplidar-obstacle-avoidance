# Lidar Radar (desktop app)

A 360 degree obstacle display for a flight controller with a lidar, like Mission Planner's
Proximity view. Works with PX4 (with the patches in `../px4_rplidar/patches`) and ArduPilot.
Tested on Windows 11 with a Pixhawk 6X, PX4 v1.17.0 and an RPLidar A2M12, 2026-11-14.

## Start

- **Windows:** double-click `Start Lidar Radar.bat`. The first time it installs `pymavlink` and
  `pyserial` if they are missing.
- **Ubuntu:** `./start_lidar_radar.sh` (needs `sudo apt install python3-tk`, and your user in the
  `dialout` group for USB).
- Or by hand: `python lidar_radar.py [COM12 | /dev/ttyACM0 | udp:14445]`.

With no argument it finds the flight controller's USB port by itself (Holybro, CubePilot,
ArduPilot and PX4 USB ids). If QGroundControl holds that port, it listens to QGC's MAVLink
forwarding on UDP 14445 instead. That needs, in QGC, Settings > MAVLink > "Enable MAVLink
forwarding" (host `localhost:14445`). Leave forwarding off when you do not use the app: QGC shows a
"UDP Link Error" when nothing listens on that port.

## What it shows

- Front is up and matches the flight controller's arrow; the vehicle icon follows the vehicle type
  the flight controller reports (quad, hexa, octo, tri, helicopter, fixed wing, VTOL, rover, boat).
- Red inside `CP_DIST` (read from the vehicle on a USB link; 2 m if unknown), yellow up to
  2 x `CP_DIST`, green beyond. Cards with the closest distance FRONT, RIGHT, BACK and LEFT.
- Header: autopilot, vehicle type, flight mode, armed state, link.
- "NO LIDAR DATA" when the scan stops for more than 1 s.
- North marker from the vehicle heading.

## Controls

| Action | How |
|---|---|
| Zoom (1.5 / 3 / 6 / 12 m) | mouse wheel, the -/+ buttons, or the + / - keys |
| Parking-sensor beeps | M, or click "sound" in the footer |
| Keep on top of other windows | T, or click "on top" in the footer |
| Quit | Q or Esc |

The window can be resized; everything scales with it.

## Notes

- On stock PX4 (without the MAVLink patch) over USB, it reads the scan through the PX4 shell
  instead, at about 10 Hz.
- Through QGC forwarding, reading `CP_DIST` gets no reply, so it shows the 2 m default.
