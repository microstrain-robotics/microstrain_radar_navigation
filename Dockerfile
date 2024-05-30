FROM ros:humble-ros-base

RUN apt-get update; exit 0

RUN mkdir -p /ros2_ws/src

COPY microstrain_radar_navigation /ros2_ws/src/microstrain_radar_navigation
COPY microstrain_inertial /ros2_ws/src/microstrain_inertial
COPY radar_velocity_estimation /ros2_ws/src/radar_velocity_estimation
COPY smartmicro_ros2_radars /ros2_ws/src/smartmicro_ros2_radars
COPY ./cyclone_dds_config.xml /
COPY docker_entrypoint.sh /usr/local/bin

RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y
RUN apt-get install libboost-all-dev -y
RUN apt-get install libtbb-dev -y
RUN apt-get install wget -y
RUN set -ex && cd /ros2_ws/src/smartmicro_ros2_radars && yes yes | ./smart_extract.sh 
RUN set -ex && cd /ros2_ws && . /opt/ros/humble/setup.sh && rosdep install --from-paths src -y --ignore-src -r -y && colcon build


ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]



