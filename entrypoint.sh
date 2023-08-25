#!/bin/bash

# Restart udev
service udev restart
udevadm control --reload-rules
udevadm trigger

# Setup ros2 environment
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"

# Execute the provided command or fall back to the default CMD
exec "$@"
