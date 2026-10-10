#!/bin/sh
# Start the RD-03D UART on Raspberry Pi 5 / QNX 8.0 (256000 8N1) as /dev/ser11.
# RP1 UART0 = GPIO14 (TXD0, pin 8) / GPIO15 (RXD0, pin 10), address 0x1f00030000, IRQ 185.
# NOTE: on the Pi 5 QSTI, /dev/ser10 is the DEBUG console UART (0x107d001000) - never use it.
# Commands follow the QNX "Configuring UART" guide for Raspberry Pi 5. Run as root.
N=${1:-11}
DEV=/dev/ser$N
msix-rp1 -m set 25l                    # RP1 MSI-X interrupt for UART0 (trailing 'l' required)
gpio-rp1 set 14,15 a4                  # GPIO14/15 -> UART0 TXD/RXD (alt function 4)
if [ -e $DEV ]; then
    if pidin arg | grep "devc-ser" | grep -- "-u$N" | grep -q "0x1f00030000"; then
        echo "$DEV already running on RP1 UART0 (GPIO14/15) - OK"
    else
        echo "ERROR: $DEV exists but is NOT RP1 UART0 (GPIO14/15):"
        pidin arg | grep devc-ser
        echo "Pick a free number, e.g.: ./start_uart.sh 12   and run aeb with -u /dev/ser12"
        exit 1
    fi
else
    devc-serpl011-rpi5 -b256000 -c50000000 -e -F -u$N 0x1f00030000,185   # -F: no flow control
    sleep 1
    [ -e $DEV ] || { echo "ERROR: driver did not create $DEV"; exit 1; }
fi
stty baud=256000 par=none bits=8 stopb=1 < $DEV
echo "UART ready: $DEV (radar on GPIO14/15)"; stty < $DEV | grep baud
