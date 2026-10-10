#!/bin/sh
# Check that the Raspberry Pi Camera Module 3 (IMX708) and the QNX Sensor
# Framework are working on the QNX Pi 5, and that the vision files are present.
# Run on the Pi, as root (su) - some commands need root.  Usage: ./check_camera.sh [unit]
U=${1:-3}
ok()   { echo "  [OK]   $*"; }
bad()  { echo "  [FAIL] $*"; }
info() { echo "  [..]   $*"; }

echo "== 1. QNX version (camera on Pi 5 needs a QNX 8.0.5 QSTI image or newer)"
use -i /proc/boot/procnto-smp-instr 2>/dev/null | grep -E "TAGID|DATE" || info "cannot read procnto info (run as root)"

echo "== 2. sensor service"
if pidin ar 2>/dev/null | grep -v grep | grep -q "sensor .*-c"; then
    ok "sensor service running:"; pidin ar | grep -v grep | grep "sensor .*-c" | sed 's/^/         /'
else
    bad "sensor service not running (it is started at boot by the image's startup script)"
fi
CONF=$(pidin ar 2>/dev/null | grep -v grep | grep "sensor .*-c" | sed 's/.*-c *\([^ ]*\).*/\1/' | head -n 1)
[ -n "$CONF" ] || CONF=/etc/config/sensor/sensor_rpi5.conf
if [ -f "$CONF" ]; then
    ok "config $CONF - units:"
    grep -E "begin SENSOR_UNIT|name *=|i2c_path|default_video" "$CONF" | sed 's/^/         /'
else
    bad "config $CONF not found"
fi

echo "== 3. camera device nodes"
ls -l /dev/sensor 2>/dev/null | sed 's/^/         /' || bad "/dev/sensor missing"
if [ -e /dev/sensor/camera$U ]; then ok "/dev/sensor/camera$U present"; else bad "/dev/sensor/camera$U missing (wrong unit? see list above)"; fi

echo "== 4. CSI receiver / camera I2C interrupts (DISP0 camera = MIPI0 + I2C6)"
msix-rp1 2>/dev/null | grep -E "I2C4|I2C6|MIPI0|MIPI1" | sed 's/^/         /' || info "msix-rp1 needs root"
pidin ar 2>/dev/null | grep -v grep | grep i2c-dwc-rpi5 | sed 's/^/         /'

echo "== 5. sensor log - was the IMX708 detected?"
slog2info -b sensor_service 2>/dev/null | grep -i -E "Camera $U|imx708|error|fail" | tail -n 12 | sed 's/^/         /'

echo "== 6. libraries"
for L in libcamapi.so libopencv_core.so libopencv_dnn.so libopencv_imgproc.so libopencv_imgcodecs.so; do
    F=$(ls /usr/lib/$L* /system/lib/$L* /usr/local/lib/$L* ./opencv_libs/$L* /data/home/qnxuser/opencv/libs/$L* 2>/dev/null | head -n 1)
    if [ -n "$F" ]; then ok "$L -> $F"; else bad "$L not found"; fi
done

echo "== 7. vision files"
[ -x ./aeb_vision ] && ok "./aeb_vision" || bad "./aeb_vision missing (make deploy-vision)"
[ -f ./models/deploy.prototxt ] && [ -f ./models/mobilenet_iter_73000.caffemodel ] && ok "./models (MobileNet-SSD)" || bad "./models/deploy.prototxt + mobilenet_iter_73000.caffemodel missing"

echo
echo "Visual check (needs a monitor on HDMI):  camera_example3_viewfinder -u $U     (Ctrl-C to stop)"
echo "Detection check:                          ./run_vision.sh -u $U -o vision_last.jpg"
