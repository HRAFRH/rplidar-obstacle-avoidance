# RPLidar driver for PX4

A PX4 driver for Slamtec RPLidar 360° scanners wired directly to a flight controller UART.
It publishes `obstacle_distance`, which PX4's Collision Prevention uses in Position mode.

Written against PX4 v1.17.0 for the RPLidar A2M12 on a Pixhawk 6X.

**Status: runs on a Pixhawk 6X with a real A2M12 (bench, 2026-11-15). Not flown; Collision Prevention not yet tested.**

- Bench result: streaming at 19.9 Hz, 184 samples per revolution, model 0x2C fw 1.32 hw 6, health 0,
  no resyncs; `obstacle_distance` fills 71 of 72 bins every 50 ms.
- **Power-up freeze (fixed):** stock PX4 on the 6X switches the 5 V peripheral rails off and on
  early in boot. A lidar powered from TELEM 5 V then hangs (0 bytes received, motor still spinning).
  `patches/fmu-v6x-5v-rails-on-at-boot.patch` keeps the rails on, as ArduPilot does; the install
  script applies it (skip with `SKIP_BOARD_PATCHES=1`). Confirmed: streaming first try after 4 of 4
  full power-offs. Powering the lidar from a separate BEC also avoids it.
- `rplidar status` shows bytes sent/received and the last bytes received, for wiring checks.
- To see the scan live: `../radar_app` (double-click `Start Lidar Radar.bat`) or the radar inside
  QGroundControl (`../qgc`).
- In QGroundControl: `patches/mavlink-obstacle-distance-raw-fallback.patch` sends the raw scan as
  OBSTACLE_DISTANCE when Collision Prevention is not running (stock PX4 sends only CP's fused data)
  and streams it on USB at 10 Hz. Enable QGC's overlay with `showObstacleDistanceOverlay=true` under
  `[FlyView]` in QGC's ini file. It draws around the drone on the map (needs GPS) and as a ring on
  the video panel (works on the bench; a configured video source makes the panel appear).

- Built into `px4_fmu-v6x_multicopter` (PX4 v1.17.0, ARM GCC 10.3.1) with no warnings; flash 96.98% used.
  The driver adds 4.4 KB. The built file is `firmware/px4_fmu-v6x_multicopter_rplidar.px4`.
- `px4_fmu-v6x_default` does not fit with that compiler (20.9 KB over), so use the multicopter variant.
- `test/host_sim` runs the real `rplidar.cpp` against a simulated RPLidar; all 10 scenarios pass
  (clang on Windows and g++ 11.4 on Ubuntu).
- Flashed and run with the real sensor on 2026-11-15 (see above).

Nothing here has flown. Bench-test with props off before any flight.

## Files

| Path | Purpose |
|---|---|
| `src/drivers/distance_sensor/rplidar/rplidar_protocol.hpp` | Protocol constants and decoding, no PX4 dependencies |
| `src/drivers/distance_sensor/rplidar/rplidar.hpp`, `rplidar.cpp` | The driver |
| `src/drivers/distance_sensor/rplidar/rplidar_main.cpp` | `rplidar start/stop/status` shell command |
| `src/drivers/distance_sensor/rplidar/module.yaml` | Parameters and serial port assignment |
| `src/drivers/distance_sensor/rplidar/Kconfig`, `CMakeLists.txt` | Build integration |
| `install_into_px4.sh` | Copies the driver into a PX4 tree and enables it for a board |
| `test/test_protocol.cpp` | Host test of the protocol decoding |
| `test/host_sim/` | Host test bench: the real driver against a simulated RPLidar |

## Wiring (A2M12 to TELEM2)

| Lidar wire | Signal | Connect to |
|---|---|---|
| Red | 5 V (4.9 to 5.2 V) | 5 V BEC, 3 A or more |
| Black | GND | BEC ground and TELEM2 pin 6 |
| Yellow | Lidar TX | TELEM2 pin 3 (RX) |
| Green | Lidar RX | TELEM2 pin 2 (TX) |
| Blue | MOTOCTL | 5 V (same as red) |

The driver does not control the motor. MOTOCTL must be tied high or the head will not spin.
Mount the lidar flat with its cable pointing to the rear of the vehicle.

## Build

On Ubuntu (or Ubuntu in WSL2), with the PX4 toolchain installed:

```bash
git clone --recursive --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git
./install_into_px4.sh PX4-Autopilot
cd PX4-Autopilot
make px4_fmu-v6x_multicopter
```

The firmware is `build/px4_fmu-v6x_multicopter/px4_fmu-v6x_multicopter.px4`. Flash it with QGroundControl
(Firmware, Advanced settings, Custom firmware file).

For another board, pass its config file, for example
`./install_into_px4.sh PX4-Autopilot boards/px4/fmu-v6c/default.px4board`.

Host test of the decoding:

```bash
cd test
g++ -std=c++14 -Wall -Wextra -I../src/drivers/distance_sensor/rplidar test_protocol.cpp -o test_protocol && ./test_protocol
```

Host test bench (needs a PX4 source tree for its maths headers, but no PX4 build):

```bash
bash test/host_sim/run_host_sim.sh ~/PX4-Autopilot
```

It replaces PX4's serial port, scheduler, parameters and uORB with stand-ins (`test/host_sim/stubs`)
and connects a simulated RPLidar. Scenarios: normal scan, motor not spinning, corrupted bytes,
health error and reset, upside-down mounting, yaw offset, nothing connected, samples split across
reads, tilted vehicle, and a slower denser scan.

## Parameters

| Parameter | Value for this setup | Meaning |
|---|---|---|
| `SENS_RPLIDAR_CFG` | TELEM 2 | Serial port of the lidar; starts the driver at boot |
| `MAV_1_CONFIG` | Disabled | TELEM2 must not also be used by MAVLink |
| `RPL_BAUD` | 256000 | A2M12 baud rate |
| `RPL_ORIENT_CFG` | Upright | Upside down if it hangs below the vehicle |
| `RPL_YAW_OFF` | 0 | Clockwise angle from vehicle front to lidar front |
| `RPL_MIN_DIST` | 0.2 | Ignore returns closer than this (airframe parts) |
| `RPL_MAX_DIST` | 12 | Beyond this is reported as free |
| `CP_DIST` | for example 2 | Enables Collision Prevention; minimum distance in metres |
| `MPC_POS_MODE` | Acceleration based | Required by Collision Prevention |

Reboot after changing them.

## Bench test (props off)

In the MAVLink shell (QGroundControl, Analyze Tools, MAVLink Console):

```
rplidar status
listener obstacle_distance
```

`rplidar status` should show the model, "state: streaming" and a scan rate of about 10 to 15 Hz.
`listener obstacle_distance` shows 72 distances in centimetres, index 0 is the front, clockwise.
Move a hand around the lidar and check that the right indices change.

## How the data is reported

- 72 bins of 5°, one message per revolution. Each bin holds the closest valid return.
- A bin the beam swept without a usable return is reported as free (maximum distance + 1).
- A bin with no samples at all is reported as unknown.
- Distances are projected onto the horizontal plane using the vehicle attitude.

## Known limits

- Legacy scan mode only. It works on every RPLidar model; the higher sample rates of
  the express and dense modes are not used.
- Only the A2M12 has been considered. The other baud rates are offered but untested.
- The model names printed by `rplidar status` come from a best-effort table.
- A surface the lidar cannot see (glass, very dark or thin objects, strong sunlight) reads as free space.
- Collision Prevention only acts in Position mode.
