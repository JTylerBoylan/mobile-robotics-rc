FROM ros:humble

RUN apt update
RUN apt install git net-tools -y

# Set work directory
WORKDIR /ros2_ws/

# Install VESC packages
COPY ./transport_drivers /ros2_ws/src/transport_drivers
COPY ./vesc /ros2_ws/src/vesc

# Install ROS dependencies
RUN cd /ros2_ws/ \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -i -y

# Install YDLidar SDK
COPY ./ydlidar_sdk /YDLidar-SDK
RUN mkdir /YDLidar-SDK/build \
    && cd /YDLidar-SDK/build \
    && cmake .. \
    && make \
    && make install

# Add YDLidar ROS2 package
COPY ./ydlidar_ros2 /ros2_ws/src/ydlidar_ros2

# Build ROS workspace
RUN cd /ros2_ws/ \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
    && . /ros2_ws/install/setup.sh

# Set up udev rules
COPY ./udev/ydlidar.rules /etc/udev/rules.d/ydlidar.rules
RUN udevadm control --reload-rules \
    && udevadm trigger