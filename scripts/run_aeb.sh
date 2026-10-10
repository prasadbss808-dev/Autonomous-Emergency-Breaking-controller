#!/bin/sh
# Start guard + AEB in full mode. Usage: ./run_aeb.sh [extra aeb options]
cd "$(dirname "$0")"
./start_uart.sh || exit 1
GUARD_OPT=""
if ! pidin -p aeb_guard >/dev/null 2>&1; then
    ./aeb_guard &
    sleep 1
fi
if pidin -p aeb_guard >/dev/null 2>&1; then
    GUARD_OPT="-G"
else
    echo "WARNING: aeb_guard not running (run as root for crash protection) - continuing without it"
fi
CONF_OPT=""
[ -f aeb_calib.conf ] && CONF_OPT="-c aeb_calib.conf"
exec ./aeb -m full $GUARD_OPT $CONF_OPT -l aeb_log.csv "$@"
