#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Copy udev rules
cp ${SCRIPT_DIR}/../udev/99-ydlidar.rules /etc/udev/rules.d/99-ydlidar.rules
cp ${SCRIPT_DIR}/../udev/99-vesc.rules /etc/udev/rules.d/99-vesc.rules

# Restart udev
udevadm control --reload-rules
udevadm trigger