#!/bin/sh
# Make sure the QNX camera service runs the REAL Camera Module 3 (IMX708)
# configuration instead of the colour-bar simulator. Falls back to the
# simulator if the real camera does not start (so the demo keeps running).
# Run as root. Usage: ./camera_setup.sh [real|sim]
WANT=${1:-real}
REAL=/system/etc/config/sensor/camera_module3.conf
SIM=/system/etc/config/sensor/sensor_demo.conf
[ -f /etc/config/sensor/sensor_rpi5.conf ] && REAL=/etc/config/sensor/sensor_rpi5.conf   # QNX 8.0.5 image
CUR=$(pidin ar 2>/dev/null | grep -v grep | grep "sensor .*-c" | sed 's/.*-c *\([^ ]*\).*/\1/' | head -n 1)
start_sensor() {
    slay sensor >/dev/null 2>&1
    sleep 1
    sensor -U 521:521 -b external -r /data/share/sensor -c "$1" &
    sleep 3
}
if [ "$WANT" = "sim" ]; then
    [ "$CUR" = "$SIM" ] || start_sensor "$SIM"
    echo "camera: simulator ($SIM)"; exit 0
fi
if [ "$CUR" = "$REAL" ] && [ -e /dev/sensor/camera1 -o -e /dev/sensor/camera3 ]; then
    echo "camera: real camera already active ($REAL)"; exit 0
fi
[ -f "$REAL" ] || { echo "camera: $REAL not found - keeping $CUR"; exit 1; }
start_sensor "$REAL"
waitfor /dev/sensor/camera1 5 >/dev/null 2>&1 || waitfor /dev/sensor/camera3 2 >/dev/null 2>&1
if [ -e /dev/sensor/camera1 -o -e /dev/sensor/camera3 ]; then
    echo "camera: real camera started ($REAL)"
    exit 0
fi
echo "camera: REAL CAMERA FAILED (check ribbon cable, CAM/DISP 0) - falling back to the simulator"
start_sensor "$SIM"
exit 2
