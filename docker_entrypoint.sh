#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE="/fastrtps.xml"

exec ros2 launch microstrain_radar_navigation cv7_ins_launch.py



