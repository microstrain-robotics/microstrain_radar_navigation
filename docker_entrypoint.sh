#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:////cyclone_dds_config.xml

exec ros2 launch microstrain_radar_navigation cv7_ins_launch.py

