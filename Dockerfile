FROM ros:humble

RUN apt update
RUN apt install git net-tools -y

WORKDIR /ros2_ws/

COPY ./transport_drivers /ros2_ws/src/transport_drivers
COPY ./vesc /ros2_ws/src/vesc

RUN cd /ros2_ws/ \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -i -y

COPY ./ydlidar_sdk /YDLidar-SDK

RUN mkdir /YDLidar-SDK/build \
    && cd /YDLidar-SDK/build \
    && cmake .. \
    && make \
    && make install

COPY ./ydlidar_ros2 /ros2_ws/src/ydlidar_ros2

RUN cd /ros2_ws/ \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
    && . /ros2_ws/install/setup.sh