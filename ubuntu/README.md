# Ubuntu steps

Use Ubuntu 24.04 or 22.04 LTS; both the PX4 v1.17 and ArduPilot setup scripts support them.
Allow 80-100 GB for the partition. Run each script with `bash` from the project folder.

The commands come from the PX4 v1.17 and ArduPilot (master) documentation and source, checked on
2026-11-14. Scripts 02 and 04 have been run successfully (in WSL Ubuntu 22.04). The commands in
01 and 03 were run by hand there. Script 05 (SITL) has not been run.

| Order | Script | What it does | Time |
|---|---|---|---|
| 1 | `01_setup_px4.sh` | Clones PX4 v1.17.0, installs its toolchain. Restart afterwards. | 20-40 min |
| 2 | `02_build_px4_rplidar.sh` | Tests the decoding, installs the driver, builds Pixhawk 6X firmware | 10-20 min |
| 3 | `03_setup_ardupilot.sh` | Clones ArduPilot, installs its toolchain. Log out and in afterwards. | 20-40 min |
| 4 | `04_build_sim_on_hardware.sh` | Builds simulation-on-hardware firmware for the Pixhawk 6X | 10 min |
| 5 | `05_sitl_rplidar.sh` | Runs SITL with a simulated RPLidar for avoidance tests | - |

## Things to know

- **The PX4 target is `px4_fmu-v6x_multicopter`.** The default variant overflows flash by about
  21 KB with Ubuntu 22.04's ARM GCC 10.3. On 24.04 (GCC 13) the default may fit; not checked.
- If you change the driver, edit it in `px4_rplidar/` and run step 2 again; it reinstalls it each time.
- **Flashing:** `bash ubuntu/02_build_px4_rplidar.sh upload` flashes over USB. Check which board
  is plugged in first. After flashing PX4, load `px4_params_backup.params` in QGroundControl
  (Parameters, Tools, Load from file), then set `SENS_RPLIDAR_CFG` to TELEM 2 and reboot.
- **Two ARM compilers:** PX4 uses the apt one in `/usr/bin` (GCC 13 on 24.04); ArduPilot installs
  GCC 10 in `/opt` and puts it first in `PATH`. Step 2 removes ArduPilot's from `PATH` for its
  own build. Whether PX4 would also build with GCC 10 was not checked.
- **Simulation on hardware has no lidar.** That build compiles the proximity library out, and the
  simulated RPLidar exists only in PC SITL. Use it to exercise the real board with a simulated
  vehicle; use step 5 to test avoidance.
- **SITL obstacles are fixed posts**, not a custom world. Custom obstacles would need either a
  polygon inclusion fence with `PRX1_TYPE 10`, or Gazebo with a bridge that is not documented.
- **Gazebo is optional.** The ArduPilot Gazebo plugin has no ready-made 360° lidar copter model,
  and it feeds lidar data to ArduPilot as single rangefinders, not as a 360° scan.
