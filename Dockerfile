FROM ros:humble-ros-base

RUN apt-get update; exit 0

RUN mkdir -p /ros2_ws/src

COPY ./microstrain_radar_navigation /ros2_ws/src
COPY ./cyclone_dds_config.xml /

RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y
RUN set -ex && cd /ros2_ws && . /opt/ros/humble/setup.sh && rosdep install --from-paths src -y --ignore-src -r -y && colcon build



