#!/bin/bash
###############################################################################
# Detect Arduino port automatically
# Checks common ports and returns the first one found
###############################################################################

for port in /dev/ttyACM1 /dev/ttyACM0 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$port" ]; then
        echo "$port"
        exit 0
    fi
done

# Default fallback
echo "/dev/ttyACM0"
