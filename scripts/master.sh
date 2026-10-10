#!/bin/sh
# =============================================================================
#  master.sh - ONE command that starts the complete QNX AEB system (as root):
#     radar UART + IMU pins + camera service + dashboard + camera detection
#     + external guard + AEB controller
#
#   ./master.sh            run with this terminal (Ctrl-C = safe stop)
#   ./master.sh bg         run in the background (untethered; ./master.sh stop)
#   ./master.sh stop       safe stop of everything
#   ./master.sh status     what is running + dashboard URL
#
#  Options (environment): SOFT=1      software PWM (if rear/front motors need it)
#                         VISION=0    without camera detection
#                         CAMERA=sim  use the simulator camera instead of the IMX708
#                         DELAY=15    seconds to wait in 'bg' mode before driving
# =============================================================================
cd "$(dirname "$0")" || exit 1
DIR=$(pwd)
MODE=${1:-run}
AEB_OPT=""
[ "${SOFT:-0}" = "1" ] && AEB_OPT="-p soft"

ip_addr() {
    ifconfig 2>/dev/null | grep "inet " | grep -v 127.0.0.1 | head -n 1 | awk '{print $2}'
}

stop_all() {
    touch stop.flag
    slay -s SIGINT aeb >/dev/null 2>&1          # AEB brakes, disables drivers, exits
    sleep 2
    slay -f aeb aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
    echo "[master] everything stopped (motor drivers disabled)"
}

status() {
    for p in aeb aeb_guard aeb_vision aeb_dash; do
        if pidin -p $p >/dev/null 2>&1; then echo "  $p: running"; else echo "  $p: -"; fi
    done
    pidin ar 2>/dev/null | grep -v grep | grep "sensor .*-c" | sed 's/^/  camera service: /'
    echo "  dashboard: http://$(ip_addr):8080"
}

check_files() {
    ok=1
    for f in aeb aeb_guard start_uart.sh aeb.conf; do
        [ -e "$f" ] || { echo "[master] MISSING: $DIR/$f"; ok=0; }
    done
    [ -x aeb_dash ] && [ -f www/index.html ] || echo "[master] note: dashboard files missing (aeb_dash, www/index.html) - running without dashboard"
    [ -x aeb_vision ] && [ -f models/mobilenet_iter_73000.caffemodel ] || { echo "[master] note: camera files missing - running without camera"; VISION=0; }
    chmod +x aeb aeb_guard aeb_dash aeb_vision *.sh 2>/dev/null
    [ $ok = 1 ]
}

start_helpers() {
    slay -f aeb aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
    rm -f stop.flag
    sleep 1
    gpio-rp1 set 2,3 a3 >/dev/null 2>&1                      # I2C1 pins (IMU)
    ./start_uart.sh || { echo "[master] radar UART failed"; exit 1; }
    if [ "${VISION:-1}" = "1" ] && [ -x ./camera_setup.sh ]; then
        ./camera_setup.sh "${CAMERA:-real}" | tee camera.log
    fi
    if [ -x ./aeb_dash ]; then
        ./aeb_dash -w ./www > dash.log 2>&1 &
        echo "[master] dashboard: http://$(ip_addr):8080"
    fi
    if [ "${VISION:-1}" = "1" ] && [ -x ./aeb_vision ]; then
        ./run_vision.sh -q > vision.log 2>&1 &
        echo "[master] camera detection running (OBJECT DETECTED lines: tail -f vision.log)"
    fi
    ./aeb_guard > guard.log 2>&1 &
    sleep 1
}

case "$MODE" in
stop)   stop_all; exit 0 ;;
status) status; exit 0 ;;
bg)
    check_files || exit 1
    echo "[master] background mode: starting in ${DELAY:-15} s - unplug the cable / put the car down"
    nohup sh -c "sleep ${DELAY:-15}; SOFT=${SOFT:-0} VISION=${VISION:-1} CAMERA=${CAMERA:-real} '$DIR/master.sh' bgrun" \
        > master.log 2>&1 &
    echo "[master] log: $DIR/master.log, AEB log: $DIR/aeb_run.log. Stop: ./master.sh stop"
    exit 0 ;;
bgrun)
    start_helpers
    RUN=0
    while [ $RUN -lt 5 ]; do
        RUN=$((RUN + 1))
        pidin -p aeb_guard >/dev/null 2>&1 || { ./aeb_guard >> guard.log 2>&1 & sleep 1; }
        echo "[master] run $RUN started $(date)"
        G=""; pidin -p aeb_guard >/dev/null 2>&1 && G="-G"
        ./aeb -m full $G -c aeb.conf -l aeb_log.csv $AEB_OPT < /dev/null >> aeb_run.log 2>&1
        RC=$?
        echo "[master] aeb exited with code $RC at $(date)"
        [ -f stop.flag ] && break
        [ $RC -eq 0 ] && break
        sleep 3
    done
    slay -f aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
    exit 0 ;;
run|*)
    check_files || exit 1
    start_helpers
    echo "[master] AEB starting (Ctrl-C = safe stop of everything)"
    G=""; pidin -p aeb_guard >/dev/null 2>&1 && G="-G"
    ./aeb -m full $G -c aeb.conf -l aeb_log.csv $AEB_OPT
    slay -f aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
    echo "[master] stopped"
    ;;
esac
