@echo off
rem Lidar Radar - Achraf Rahibe. Double-click to start.
rem Finds the flight controller's USB port by itself; if QGroundControl holds it,
rem it listens to QGC's MAVLink forwarding (UDP 14445) instead.
cd /d "%~dp0"
python -c "import pymavlink, serial" 2>nul || (
    echo Installing pymavlink and pyserial...
    python -m pip install -r requirements.txt
)
python lidar_radar.py %*
if errorlevel 1 pause
