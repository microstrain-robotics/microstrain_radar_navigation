ARG ROS_VERSION="humble"
ARG WORKSPACE_DIR="/ros2_ws"

FROM ros:${ROS_VERSION}-ros-base as builder

#Installing required packages
ENV DEBIAN_FRONTEND="noninteractive"
RUN set -ex \
    && apt-get update && apt-get install -y \
    libboost-all-dev \
    libtbb-dev \
    wget

#Copying Packages into the Workspace
ARG ROS_VERSION
ARG WORKSPACE_DIR
COPY microstrain_radar_navigation ${WORKSPACE_DIR}/src/microstrain_radar_navigation
COPY microstrain_inertial ${WORKSPACE_DIR}/src/microstrain_inertial
COPY radar_velocity_estimation ${WORKSPACE_DIR}/src/radar_velocity_estimation
COPY smartmicro_ros2_radars ${WORKSPACE_DIR}/src/smartmicro_ros2_radars

#Installing Dependencies and Building the required Packages
RUN set -ex \
    && cd ${WORKSPACE_DIR} \
    && . /opt/ros/${ROS_VERSION}/setup.sh \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rm -rf ${WORKSPACE_DIR}/src/smartmicro_ros2_radars/smart_rviz_plugin \
    && rosdep install --from-paths src -y --ignore-src -r -y \
    && cd ${WORKSPACE_DIR}/src/smartmicro_ros2_radars \
    && yes yes | ./smart_extract.sh \
    && cd ../.. \
    && colcon build --cmake-args "-DCMAKE_BUILD_TYPE=RELEASE"

#Required for Disabling Shared Memory
COPY fastrtps.xml /

#Required for the Smartmicro Node to set up the CAN Interface
USER root

COPY docker_entrypoint.sh /usr/local/bin
ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]



