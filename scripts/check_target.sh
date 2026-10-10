#!/bin/sh
# Pre-flight checks on the Pi before running the AEB controller.
echo "== QNX / CPUs ==";       uname -a; pidin info | head -5
echo "== GPIO server ==";      ls -l /dev/gpio/msg 2>/dev/null || echo "MISSING: start the rpi_gpio resource manager"
echo "== I2C bus ==";          ls -l /dev/i2c1 2>/dev/null || echo "MISSING: /dev/i2c1"
echo "== Radar UART ==";       ls -l /dev/ser11 2>/dev/null || echo "MISSING: run ./start_uart.sh"
echo "== Guard ==";            pidin -p aeb_guard 2>/dev/null | tail -n +2 || echo "aeb_guard not running"
echo "== Resource managers =="; pidin -f a | grep -E "rpi_gpio|i2c|devc-ser|aeb" 
