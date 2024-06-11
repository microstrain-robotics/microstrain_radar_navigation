## Description
An example of interfacing the CV7-INS with a GNSS receiver and a Radar sensor. <br />
The integration utilizes GNSS position and velocity data from the GNSS receiver, combined with body frame velocity data from the radar point cloud, as aiding measurements for the CV7-INS. The implementation is developed using the ROS2 framework.

## Hardware
The following hardware components are utilized in this setup:
 * [`CV7-INS`](https://www.microstrain.com/inertial-sensors/3dm-cv7-ins) - The Inertial Navigation System
 * [`Ublox ZED-F9P`](https://www.u-blox.com/en/product/zed-f9p-module) - The GNSS Module
 * [`Smartmicro DRVEGRD 152`](https://www.smartmicro.com/automotive-radar/drvegrd-line) - The Automotive Radar Sensor
 * [`PCAN-USB FD Adapter`](https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1) - Allows the Connection of the Radar CAN Network to a Comupter through USB <br />
 
The use of the PCAN adapter is recommended, as widely used Linux kernels already include drivers for PEAK-System's CAN interfaces. Using a different adapter may require additional or alternative steps to configure the interface and set the baudrate.


## Contents
  * [`microstrain_inertial`](https://github.com/LORD-MicroStrain/microstrain_inertial) -- Driver software for the Microstrain Inertial Sensors (Submoduled) <br />
  * [`smartmicro_ros2_radars`](https://github.com/smartmicro/smartmicro_ros2_radars/tree/master) -- Driver software for the Smartmicro Radars (Submoduled) <br />
  * [`radar_velocity_estimation`](https://github.com/microstrain-robotics/radar_velocity_estimation) -- Software to Estimate Body Frame Velocities from the Radar Point Cloud (Submoduled) <br />
  * `microstrain_radar_navigation` -- The Example that shows how to interact with all the different nodes.

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
2. Clone the Repository into your workspace:
   ```
   git clone https://github.com/microstrain-robotics/microstrain_radar_navigation.git --recursive
   ```
2. Navigate to the Workspace and Build the Image
   ```
   cd ~/your_workspace/src/microstrain_radar_navigation
   docker build -t microstrain_radar_navigation .
   ```
   
## Launch Instructions
1. Make sure the sensors are connected to their corresponding ports
   * TTYACM0 - CV7_INS 
   * TTYACM1 - Ublox ZED-F9P <br />
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



