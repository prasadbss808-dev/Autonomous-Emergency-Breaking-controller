#!/bin/sh
# check_motors.sh - finds why the wheels turn the wrong way, then runs the motor test
#   ./check_motors.sh        only check
#   ./check_motors.sh test   check, then run the motor test (car on a stand!)
cd "$(dirname "$0")" || exit 1
BAD=0
echo "== 1. other programs using the motor pins (must be none) =="
# two programs writing the same GPIO pins make the motors do strange things
if pidin -f a 2>/dev/null | grep -E "aeb|gpio" | grep -v -E "rpi_gpio|aeb_dash|grep|check_motors"; then
    echo "   ^ stop these first:  ./master.sh stop ; slay -f aeb"
    BAD=1
else
    echo "   none - ok"
fi
echo "== 2. pin settings in every .conf file in $(pwd) =="
# correct pins: front RPWM 12 LPWM 22 EN 23, rear RPWM 13 LPWM 27 EN 24
for f in *.conf; do
    [ -f "$f" ] || continue
    grep -n -E "^[[:space:]]*pin_" "$f" | while IFS= read -r line; do
        key=$(echo "$line" | sed 's/^[0-9]*://; s/=.*//; s/[[:space:]]//g')
        val=$(echo "$line" | sed 's/.*=//; s/#.*//; s/[[:space:]]//g')
        case "$key" in
        pin_front_rpwm) want=12 ;;
        pin_front_lpwm) want=22 ;;
        pin_front_en) want=23 ;;
        pin_rear_rpwm) want=13 ;;
        pin_rear_lpwm) want=27 ;;
        pin_rear_en) want=24 ;;
        *) want="?" ;;
        esac
        if [ "$val" = "$want" ]; then
            echo "   $f: $key = $val  ok"
        else
            echo "   $f: $key = $val  WRONG (must be $want)"
        fi
    done
done
# second pass only to set the exit status (the loop above runs in a subshell)
if grep -h -E "^[[:space:]]*pin_" *.conf 2>/dev/null | tr -d ' \t' | grep -v -x -E \
    "pin_front_rpwm=12|pin_front_lpwm=22|pin_front_en=23|pin_rear_rpwm=13|pin_rear_lpwm=27|pin_rear_en=24" | grep -q .; then
    echo "   -> a .conf file has wrong motor pins: copy the good aeb.conf from the zip again"
    BAD=1
fi
echo "== 3. motor driver programs on the Pi =="
ls -l aeb aeb.conf 2>/dev/null
ls -l /dev/gpio/msg >/dev/null 2>&1 && echo "   rpi_gpio running - ok" || { echo "   rpi_gpio NOT running"; BAD=1; }
echo "== 4. what the AEB program really uses (printed at start) =="
# dry run: motors stay off, "quit" makes it exit straight after printing the pin line
echo quit | ./aeb -m sim -s 1 -p none -c aeb.conf -q 2>&1 | grep -E "motor pins|parameters" | head -3
echo "   correct: front RPWM 12 LPWM 22 EN 23 | rear RPWM 13 LPWM 27 EN 24"
[ $BAD = 1 ] && echo "RESULT: problem found above - fix it first" || echo "RESULT: software side is ok"
if [ "$1" = "test" ]; then
    echo
    echo "== 5. motor test: front axle, rear axle, both, brake, coast =="
    echo "   put the car on a stand, master switch ON. Ctrl-C = stop"
    ./aeb -m motor -y -c aeb.conf
    echo
    echo "if a whole AXLE still turns backward with correct pins -> the wires were swapped:"
    echo "   swap that driver's M+ and M- output wires (power OFF first)"
    echo "   or check RPWM/LPWM wires at the Pi: RPWM on pin 32/33, LPWM on pin 15/13"
    echo "if only ONE WHEEL turns backward -> swap only that motor's two wires"
fi
