#!/bin/sh
# Safe stop of an untethered or background run: SIGINT -> the AEB brakes,
# disables the drivers and exits cleanly; then the helper processes stop.
cd "$(dirname "$0")"
touch stop.flag
slay -s SIGINT aeb >/dev/null 2>&1
sleep 2
slay -f aeb_guard aeb_vision aeb_dash >/dev/null 2>&1
echo "stopped. last lines of aeb_run.log:"
tail -n 10 aeb_run.log 2>/dev/null
echo "--- autostart.log:"
tail -n 6 autostart.log 2>/dev/null
