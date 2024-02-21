# Description
An Example of Interfacing the CV7-INS with a Ublox ZED-F9P and a Radar for Navigation.

# Common
TTYACM0 - CV7_INS <br />
TTYACM1 - Ublox ZED-F9P

git clone https://github.com/microstrain-robotics/microstrain_radar_navigation.git --recursive

# Manual Build and Run
Copy the Repo to <ROS2_WORKSPACE>/src <br />
cd .. <br />
rosdep install --from-paths src -y --ignore-src -r -y <br />
colcon build <br />
<br />
sudo apt-get install ros-<distro>-rmw-cyclonedds-cpp <br />
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp <br />
export CYCLONEDDS_URI=file:////<ROS2_WORKSPACE>/src/microstrain_radar_navigation/cyclone_dds_config.xml <br />
<br />
ros2 launch microstrain_radar_navigation cv7_ins_launch.py

# Docker
docker build -t <IMAGE_NAME> . <br />
xhost local:root <br />
docker run --rm -it --env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged -v /dev:/dev --net=host <IMAGE_NAME>
