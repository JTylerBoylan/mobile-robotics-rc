#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Copy udev rules
cp ${SCRIPT_DIR}/../udev/ydlidar.rules /etc/udev/rules.d/ydlidar.rules

# Restart udev
udevadm control --reload-rules
udevadm trigger