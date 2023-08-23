FROM ros:humble

RUN apt update
RUN apt install git net-tools -y

RUN mkdir -p /ros2_ws/src/

RUN cd /ros2_ws/src/ \
    && git clone https://github.com/ros-drivers/transport_drivers.git \
    && git clone -b ros2 https://github.com/f1tenth/vesc.git

RUN cd /ros2_ws/ \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -i -y