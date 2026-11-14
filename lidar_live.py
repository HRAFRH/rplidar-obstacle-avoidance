"""Live view of the RPLidar proximity data coming from an ArduPilot flight controller.

Usage:  python lidar_live.py [COM port]      (Ctrl+C to stop)
Read-only: it only listens, it does not change any parameter.
"""
import sys
import time

import serial.tools.list_ports as list_ports
from pymavlink import mavutil

NAMES = ['front', 'front-right', 'right', 'back-right',
         'back', 'back-left', 'left', 'front-left']
PROXIMITY_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY  # 0x4000000


def find_port():
    for p in list_ports.comports():
        if 'VID:PID=1209:5740' in p.hwid and 'SLCAN' not in p.description:
            return p.device
    return None


port = sys.argv[1] if len(sys.argv) > 1 else find_port()
if not port:
    sys.exit('No ArduPilot flight controller found on USB.')

print(f'Connecting to {port} ...')
m = mavutil.mavlink_connection(port, baud=115200)
if not m.wait_heartbeat(timeout=15):
    sys.exit('No heartbeat from the flight controller.')

# Ask for DISTANCE_SENSOR (132) at 10 Hz and SYS_STATUS (1) at 2 Hz.
m.mav.command_long_send(m.target_system, 1, 511, 0, 132, 100000, 0, 0, 0, 0, 0)
m.mav.command_long_send(m.target_system, 1, 511, 0, 1, 500000, 0, 0, 0, 0, 0)

dist = {}      # sector -> (distance in m, time received)
healthy = None
last_draw = 0.0

try:
    while True:
        msg = m.recv_match(type=['DISTANCE_SENSOR', 'SYS_STATUS', 'STATUSTEXT'],
                           blocking=True, timeout=0.2)
        now = time.time()
        if msg:
            kind = msg.get_type()
            if kind == 'DISTANCE_SENSOR' and msg.orientation < 8:
                dist[msg.orientation] = (msg.current_distance / 100.0, now)
            elif kind == 'SYS_STATUS':
                healthy = bool(msg.onboard_control_sensors_health & PROXIMITY_BIT)
            elif kind == 'STATUSTEXT':
                print('board:', msg.text)
        if now - last_draw >= 0.3:
            last_draw = now
            lines = [time.strftime('%H:%M:%S') + f'   proximity healthy: {healthy}']
            for i, name in enumerate(NAMES):
                if i in dist and now - dist[i][1] < 1.0:
                    d = dist[i][0]
                    bar = '#' * min(40, int(d * 4))
                    lines.append(f'  {name:12s} {d:5.2f} m  {bar}')
                else:
                    lines.append(f'  {name:12s}   --     (nothing in range / no data)')
            print('\n'.join(lines) + '\n', flush=True)
except KeyboardInterrupt:
    print('stopped')
