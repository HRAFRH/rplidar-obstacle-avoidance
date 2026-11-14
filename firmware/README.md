# Built firmware for the Pixhawk 6X

Both files were built on 2026-11-14 in WSL Ubuntu 22.04. **Neither has been flashed or run yet.**
Both are for the Pixhawk 6X only (board id 53). Props off for every first run.

## px4_fmu-v6x_multicopter_rplidar.px4

PX4 v1.17.0, official `multicopter` variant, plus the RPLidar driver from `../px4_rplidar`.

- Flash: QGroundControl, Vehicle Setup, Firmware, plug the board in, tick "Advanced settings",
  choose "Custom firmware file".
- Then load `../px4_params_backup.params` (Parameters, Tools, Load from file) to restore the
  airframe and calibrations the board had under PX4.
- Set `SENS_RPLIDAR_CFG` to TELEM 2 and keep `MAV_1_CONFIG` disabled. Reboot.
- Check in the MAVLink console: `rplidar status`, then `listener obstacle_distance`.
- Only after that works: `CP_DIST` (for example 2) and `MPC_POS_MODE` acceleration based.

## cubepilot_cubeorangeplus_rplidar.px4

PX4 v1.17.0 for the **Cube Orange Plus** (board id 1063), default variant, with the RPLidar driver
and the patches in `../px4_rplidar/patches`. Flash 99.40% used. Flashed and tested on 2026-11-14:
lidar streams on TELEM2 (`/dev/ttyS1`), also after a cold start.

- Set `SYS_AUTOSTART` (4001 = quad X) and reboot, then `SENS_RPLIDAR_CFG` = 102 (TELEM 2),
  `MAV_1_CONFIG` = 0, `CP_DIST` (e.g. 2). Reboot.
- To return to ArduPilot: flash ArduCopter for CubeOrangePlus and load
  `../ardupilot/cubeorangeplus_full_backup_before_px4.param`.

## arducopter_Pixhawk6X_SIM_ON_HARDWARE.apj

ArduCopter 4.8.0-dev (git 98f6b839) built with `sitl-on-hw.py`, quad frame. The board runs a
simulated vehicle: IMU, GPS and compass are simulated.

- **It is not flight firmware. Never fly with it.**
- **It has no proximity support**, so it cannot show the lidar or test obstacle avoidance.
- It is development code, newer than the 4.7.2 stable release on the board.
- Motor outputs stay off unless `SIM_OH_MASK` is set. Keep props off regardless.
- It shares parameter storage with normal firmware. Before flashing it, save the board's
  parameters. After going back to normal ArduCopter, reset them (`FORMAT_VERSION` to 0, reboot)
  and reload `../ardupilot/rplidar_a2m12_telem2.param`.
- Flash: Mission Planner, Setup, Install Firmware, "Load custom firmware".

## Going back to normal ArduCopter

Mission Planner, Setup, Install Firmware, choose Copter (stable) for Pixhawk6X.
