#!/bin/sh
# Untethered run (no Ethernet / no terminal needed). Run as root:
#   nohup ./autostart.sh 15 > /dev/null 2>&1 &        then unplug the Ethernet cable
# Arguments: [delay_s] [extra aeb options, e.g. -p soft]
# Environment: VISION=0 to skip the camera, CAMERA=sim to use the simulator camera,
#              DASH=0 to skip the dashboard.
# Logs: autostart.log (supervisor), aeb_run.log (AEB), vision.log, dash.log, aeb_log.csv.
cd "$(dirname "$0")"
DELAY=${1:-15}
[ $# -gt 0 ] && shift
echo "autostart: waiting ${DELAY}s (put the car down, step away)" > autostart.log
sleep "$DELAY"
slay -f aeb aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
rm -f stop.flag
sleep 1
[ "${VISION:-1}" = "1" ] && ./camera_setup.sh "${CAMERA:-real}" >> camera.log 2>&1
gpio-rp1 set 2,3 a3 >> autostart.log 2>&1
./start_uart.sh >> autostart.log 2>&1 || { echo "autostart: radar UART failed" >> autostart.log; exit 1; }
[ -x ./aeb_dash ]   && [ "${DASH:-1}" = "1" ]   && { ./aeb_dash -w ./www > dash.log 2>&1 & }
[ -x ./aeb_vision ] && [ "${VISION:-1}" = "1" ] && { ./run_vision.sh -q > vision.log 2>&1 & }
RUN=0
while [ $RUN -lt 5 ]; do
    RUN=$((RUN + 1))
    pidin -p aeb_guard >/dev/null 2>&1 || { ./aeb_guard >> autostart.log 2>&1 & sleep 1; }
    GUARD_OPT=""
    pidin -p aeb_guard >/dev/null 2>&1 && GUARD_OPT="-G"
    echo "autostart: run $RUN: aeb $GUARD_OPT -c aeb.conf $* ($(date))" >> autostart.log
    ./aeb -m full $GUARD_OPT -c aeb.conf -l aeb_log.csv "$@" < /dev/null >> aeb_run.log 2>&1
    RC=$?
    echo "autostart: aeb exited with code $RC ($(date))" >> autostart.log
    [ -f stop.flag ] && { rm -f stop.flag; echo "autostart: stopped by stop_aeb.sh" >> autostart.log; break; }
    [ $RC -eq 0 ] && break            # clean exit (quit / Ctrl-C / SIGTERM)
    sleep 3                           # unexpected exit: restart (max 5 runs)
done
slay -f aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
echo "autostart: finished" >> autostart.log
