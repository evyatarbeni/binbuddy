#!/bin/bash
# Detect Arduino port automatically

for port in /dev/ttyACM1 /dev/ttyACM0 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$port" ]; then
        echo "$port"
        exit 0
    fi
done

echo "/dev/ttyACM0"  # Default fallback
