FROM ros:humble-ros-base

RUN apt-get update; exit 0

RUN mkdir -p /ros2_ws/src

COPY microstrain_radar_navigation /ros2_ws/src/microstrain_radar_navigation
COPY microstrain_inertial /ros2_ws/src/microstrain_inertial
COPY ms_radar_processing /ros2_ws/src/ms_radar_processing
COPY ./cyclone_dds_config.xml /
COPY docker_entrypoint.sh /usr/local/bin

RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y
RUN apt-get install libboost-all-dev -y
RUN apt-get install libtbb-dev -y
RUN set -ex && cd /ros2_ws && . /opt/ros/humble/setup.sh && rosdep install --from-paths src -y --ignore-src -r -y && colcon build

ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]



