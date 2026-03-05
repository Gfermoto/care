#!/bin/bash
# Подключение радара LD2450 к WSL через usbipd
# CH340, CP2102, FT232
#
# ./scripts/connect_radar.sh [BUSID]
# RADAR_BUSID=3-4 ./scripts/connect_radar.sh

BUSID=${1:-${RADAR_BUSID:-"5-6"}}

echo "Connecting radar to WSL..."

powershell.exe -Command "
    \$e = \$ErrorActionPreference; \$ErrorActionPreference = 'SilentlyContinue'
    usbipd bind --busid ${BUSID}
    \$ErrorActionPreference = \$e
    usbipd attach --wsl --busid ${BUSID}
"

echo "Waiting 10 sec..."
sleep 10

powershell.exe usbipd list
echo ""

if [ -e /dev/ttyUSB0 ]; then
    RADAR_PORT="/dev/ttyUSB0"
elif [ -e /dev/ttyUSB1 ]; then
    RADAR_PORT="/dev/ttyUSB1"
fi

if [ -n "$RADAR_PORT" ]; then
    echo "Port $RADAR_PORT ready."
    if [ "$RADAR_PORT" = "/dev/ttyUSB0" ]; then
        echo "  python3 scripts/radar.py"
    else
        echo "  python3 scripts/radar.py -p $RADAR_PORT"
    fi
else
    echo "Ports /dev/ttyUSB0, /dev/ttyUSB1 not found."
    echo "Check: ls -la /dev/ttyUSB*"
    echo "Manual: usbipd bind --busid <ID> ; usbipd attach --wsl --busid <ID>"
fi
