## Description
This example demonstrates the integration of a MicroStrain CV7-INS with a GNSS receiver and a mmWave radar sensor. It utilizes GNSS position and velocity data from the GNSS receiver, combined with body frame velocity data from the radar point cloud, as aiding measurements for the CV7-INS. The implementation is developed using the ROS2 framework.

## Hardware
The following hardware components are utilized in this setup:
 * [`MicroStrain CV7-INS`](https://www.microstrain.com/inertial-sensors/3dm-cv7-ins) - Core Inertial Navigation System with onboard IMU and EKF for sensor fusion
 * [`Ublox ZED-F9P`](https://www.u-blox.com/en/product/zed-f9p-module) - GNSS Module
 * [`Smartmicro DRVEGRD 152`](https://www.smartmicro.com/automotive-radar/drvegrd-line) - Automotive mmWave radar sensor
 * [`PCAN-USB FD Adapter`](https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1) - USB CAN-FD adapter to interface with the radar CAN bus through USB<br />
 
The use of the PCAN adapter is recommended, as widely used Linux kernels already include drivers for PEAK-System's CAN interfaces. Using a different adapter may require additional or alternative steps to configure the interface and set the baudrate.


## Contents
  * [`microstrain_inertial`](https://github.com/LORD-MicroStrain/microstrain_inertial) -- ROS driver for the CV7-INS(Submoduled) <br />
  * [`smartmicro_ros2_radars`](https://github.com/smartmicro/smartmicro_ros2_radars/tree/master) -- ROS driver for the Smartmicro radar (Submoduled) <br />
  * [`radar_velocity_estimation`](https://github.com/microstrain-robotics/radar_velocity_estimation) -- Software to process the raw radar point cloud and estimate a bodyframe velocity measurement (Submoduled) <br />
  * `microstrain_radar_navigation` -- Example package that demonstrates how to configure and launch all nodes

## Building from Source
1. Install [ROS2](https://docs.ros.org/en/humble/Installation.html) and [Create a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. Clone the Repository into your workspace:
   ```
   git clone https://github.com/microstrain-robotics/microstrain_radar_navigation.git --recursive
   ```
3. Install rosdeps for all the packages: `rosdep install --from-paths ~/your_workspace/src -i -r -y`
4. Run the SmartExtract Script
   ```
   cd ~/your_workspace/src/microstrain_radar_navigation/smartmicro_ros2_radars/
   ./smart_extract.sh
   ```
5. Build your workspace
   ```
   cd ~/your_workspace
   colcon build
   ```
6. Source ROS2, the Workspace and Set Environment Variables
   ```
   source /opt/ros/humble/setup.bash
   source /your_workspace/install/setup.bash
   ```
   The source commands need to be run on every terminal before launching the node or these lines can just be added to the .bashrc

## Building the Docker Image
1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/)
2. Clone the repository into your workspace:
   ```
   git clone https://github.com/microstrain-robotics/microstrain_radar_navigation.git --recursive
   ```
2. Navigate to the workspace and build the Docker image
   ```
   cd ~/your_workspace/src/microstrain_radar_navigation
   docker build -t microstrain_radar_navigation .
   ```
   
## Launch Instructions
1. Make sure the sensors are connected to their corresponding ports (or change port names in the relevant [configuration YAML](microstrain_radar_navigation/config) files)
   * TTYACM0 - CV7-INS 
   * TTYACM1 - Ublox ZED-F9P
   * The SmartMicro DRVEGRD 152 is connected using a PCAN-USB FD Adapter.

2. Set the Baudrate for the PCAN Adapter
    ```
    sudo ip link set can0 up type can bitrate 500000
    ```
3. Launch the Node (Building from Source)
   ```
   ros2 launch microstrain_radar_navigation cv7_ins_launch.py
   ```
4. Launch the Node (Using the Docker Image)
   ```
   xhost local:root
   docker run --rm -it --env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged -v /dev:/dev --net=host microstrain_radar_navigation
   ```
   
## License
Different packages in this repo are released under different licenses. For more information, see the LICENSE files in each of the package directories.



