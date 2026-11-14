#!/usr/bin/env bash
# Lidar Radar - Achraf Rahibe.  Usage: ./start_lidar_radar.sh [/dev/ttyACM0 | udp:14445]
# Finds the flight controller's USB port by itself when no argument is given.
# On Ubuntu: sudo apt install python3-tk ; your user must be in the dialout group for USB.
cd "$(dirname "$0")"
python3 -c "import pymavlink, serial, tkinter" 2>/dev/null || python3 -m pip install --user -r requirements.txt
exec python3 lidar_radar.py "$@"
