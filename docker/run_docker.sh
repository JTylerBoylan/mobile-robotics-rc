#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Build the docker image
docker build -t mobirobo "${SCRIPT_DIR}"

# Start the Docker container with the current directory mounted to /app, and automatically remove the container when it is stopped or exited
docker run -it \
    --rm \
    --net host \
    --privileged \
    -v /dev:/dev \
    --mount type=bind,source="${SCRIPT_DIR}/../ydlidar_ros2",target=/ros2_ws/src/ydlidar_ros2 \
    mobirobo