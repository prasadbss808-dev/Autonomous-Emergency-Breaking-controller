#!/bin/sh
# Everything, with a terminal (as root):  ./start_all.sh [aeb options, e.g. -p soft]
#   dashboard (http://<pi-ip>:8080) + camera detection + guard + AEB (foreground).
#   Ctrl-C stops the AEB safely and then the helpers.
#   VISION=0 ./start_all.sh   -> without camera;  CAMERA=sim ./start_all.sh -> simulator camera
cd "$(dirname "$0")"
slay -f aeb aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
sleep 1
[ "${VISION:-1}" = "1" ] && ./camera_setup.sh "${CAMERA:-real}" >> camera.log 2>&1
gpio-rp1 set 2,3 a3 >/dev/null 2>&1
./start_uart.sh || exit 1
[ -x ./aeb_dash ]   && { ./aeb_dash -w ./www > dash.log 2>&1 & }
[ -x ./aeb_vision ] && [ "${VISION:-1}" = "1" ] && { ./run_vision.sh -q > vision.log 2>&1 & }
./aeb_guard &
sleep 1
GUARD_OPT=""
pidin -p aeb_guard >/dev/null 2>&1 && GUARD_OPT="-G"
IP=$(ifconfig 2>/dev/null | grep "inet " | grep -v 127.0.0.1 | head -n 1 | awk '{print $2}')
echo "Dashboard: http://${IP:-<pi-ip>}:8080    camera log: vision.log (OBJECT DETECTED lines)"
./aeb -m full $GUARD_OPT -c aeb.conf -l aeb_log.csv "$@"
slay -f aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
