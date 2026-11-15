# MTF-02P: switch to MAVLink for PX4 (lab checklist)

Pixhawk 6X side is ALREADY DONE (2026-11-15): TELEM3 = MAVLink instance 1, 115200, Normal mode,
flow control off, EKF2_GPS_CTRL 0, EKF2_OF_CTRL 1, EKF2_RNG_CTRL 1, SENS_FLOW_ROT 0.
The sensor was sending ~1.5 KB/s on TELEM3 but not as MAVLink. Only the sensor needs changing.

## Tools (two ways, pick one)
- **Web app, nothing to install:** https://micoair.com/assistant/ in Chrome or Edge.
- **Desktop app:** `MicoAssistant\MicoAssistant_EN_V0.9.23.exe` (official, from micoair.com/downloads,
  not code-signed so Windows SmartScreen may warn: "More info" > "Run anyway").
  If it crashes (French Windows), use `MicoAssistant_EN_V0.9.23_NoChart.exe`.

## Wiring: MTF-02P to USB-TTL adapter (unplug it from the Pixhawk first)
| MTF-02P | Adapter |
|---|---|
| 5 V | 5 V |
| GND | GND |
| TX | RX |
| RX | TX |
Adapter logic level 3.3 V if it has a jumper. If Windows shows no COM port, install the driver
for the chip (CP210x or CH340) from https://micoair.com/downloads/.

## Steps
1. Plug in the adapter. Device Manager > Ports: note the COM number.
2. Open MicoAssistant. Select that COM port, baud **115200**, Connect.
   You should see live flow and distance values (move a hand under the sensor).
3. Setup page: protocol **Mavlink_PX4** (NOT Mavlink_APM, Micolink or MSP). Baud stays 115200.
4. Click **Write**. Disconnect, reconnect, check it still says Mavlink_PX4.
5. Back on the Pixhawk TELEM3: 5 V pin 1, sensor TX to pin 3, sensor RX to pin 2, GND pin 6.

## Check on the Pixhawk (Claude does this, or QGC MAVLink Console)
- `mavlink status`: instance on /dev/ttyS1 must list "Received Messages" with a sysid.
- `listener distance_sensor` and `listener vehicle_optical_flow`: must publish.
- Hand test: move the board, flow follows; lift it, distance follows.
- Then: orientation check, height limits (SENS_FLOW_MINHGT / MAXHGT), Position mode without GPS.
