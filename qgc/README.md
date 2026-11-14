# Lidar radar inside QGroundControl

`LidarRadarInstrument.qml` draws the 360 degree lidar scan in QGC's Fly view, next to QGC's own
attitude and compass instrument. No QGC rebuild is needed: QGC loads its instrument panel from the
file named in its settings, and accepts a local file there. Tested with QGroundControl Daily
v5.0.3-1068 on Windows, 2026-11-14, with PX4 v1.17.0 plus the patches in `../px4_rplidar/patches`.

What it shows:
- Front is up and matches the flight controller's arrow (the scan is in the body frame).
- Red inside `CP_DIST` (read from the vehicle; 2 m if missing), yellow up to 2 x `CP_DIST`, green beyond.
- The closest distance on each side (FRONT, RIGHT, BACK, LEFT), the closest overall, and a north
  marker from the vehicle heading.
- "NO LIDAR DATA" when no OBSTACLE_DISTANCE has arrived for 1 s.

It reads the OBSTACLE_DISTANCE message. Stock PX4 only sends it while Collision Prevention runs;
`../px4_rplidar/patches/mavlink-obstacle-distance-raw-fallback.patch` makes PX4 send it always.

## Install

Close QGroundControl, then edit its settings file
(Windows: `%APPDATA%\QGroundControl\QGroundControl Daily.ini`; Linux: `~/.config/QGroundControl/`).
Under `[FlyView]`:

```
instrumentQmlFile2=file:///D:/Dev/LIDAR/qgc/LidarRadarInstrument.qml
```

Use the real path of the file (on Linux e.g. `file:///home/<user>/LIDAR/qgc/LidarRadarInstrument.qml`).
QGC rewrites this file while it runs, so edit it only while QGC is closed.

Optional, also used on the bench:
- `[FlyView] showObstacleDistanceOverlay=true`: QGC's own red ring on the map (needs GPS) and video.
- Video from the SIYI A8 mini through the SIYI air unit (PC on the 192.168.144.x network):
  `[Video] videoSource=RTSP Video Stream` and `rtspUrl=rtsp://192.168.144.25:8554/main.264`
  (the A8 mini's default address; check it in SIYI's tool if you changed it). QGC's own obstacle
  ring is then drawn over the camera picture.
- `[General] forwardMavlink=true`: forwards MAVLink to localhost:14445 for `../radar_app`. Leave it
  false otherwise: QGC shows "UDP Link Error" when nothing listens there.

## Using the widget

- It sits above the attitude instrument at the same width. Drag it anywhere; double-click it to
  put it back. QGC remembers the position and zoom (section `[LidarRadarWidget]` of the ini).
- Mouse wheel or the -/+ buttons: range 1.5 / 3 / 6 / 12 m.
- The vehicle icon follows the vehicle type (quad, hexa, octo, tri, helicopter, plane, VTOL, rover, boat).
- It is drawn on the Fly view's top level so it can be dragged; other pages cover it as usual.

## Going back to QGC's normal instrument

Close QGC and set `instrumentQmlFile2=qrc:/qml/QGroundControl/FlightMap/Widgets/VerticalCompassAttitude.qml`,
or right-click the instrument in the Fly view and pick one from the list.
