#!/bin/sh
# Start the dashboard web server. Open http://<pi-ip>:8080 in a browser
# (laptop or phone on the same network: Ethernet or Wi-Fi).
cd "$(dirname "$0")"
exec ./aeb_dash -p "${1:-8080}" -w ./www
