import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_INS_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_radar_navigation'), 'config', 'cv7_ins', 'cv7_ins.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_radar_navigation'), 'config', 'cv7_ins', 'display.rviz')

_CONFIG_DIRECTORY = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_radar_navigation'), 'config')
_UBLOX_CONFIG_FILE = os.path.join(_CONFIG_DIRECTORY, 'zed_f9p.yaml')
_SMARTMICRO_CONFIG_FILE = os.path.join(_CONFIG_DIRECTORY, 'radar_params.yaml')

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _CV7_INS_PARAMS_FILE,
        'namespace': '/cv7_ins',
	      'debug': 'true',
      }.items()
    ),
    
    # Ublox ZED-F9P Node
    Node(
      package='ublox_gps',
      executable='ublox_gps_node',
      output='both',
      parameters=[_UBLOX_CONFIG_FILE],
      remappings=[
        ('/fix','/cv7_ins/ext/llh_position'),
        ('/fix_velocity','/cv7_ins/ext/velocity_enu')
      ]),

    # SmartMicro Radar Node
    Node(
      package='umrr_ros2_driver',
      executable='smartmicro_radar_node_exe',
      name='smart_radar',
      parameters=[_SMARTMICRO_CONFIG_FILE]),

    # Radar Processing Node  
    Node(
      package='radar_velocity_estimation_node',
      executable='radar_velocity_estimation_node',
      output='screen'),    

    # Publish a static transform for where the our gps is mounted on base_link.
    # You should replace this with actual transforms for where your aiding sensors are
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0.15",
          "--y", "-0.45",
          "--z", "1.58",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "gps"
        ]
    ),

    # Publish a static transform for where the CV7-INS is mounted on base_link.
    # Unless the CV7-INS is mounted exactly at base_link, you should change this to be accurate to your setup
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "-0.37",
          "--y", "-0.55",
          "--z", "0.63",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "cv7_ins_link"
        ]
    ),

    # Publish a static transform for where the our radar is mounted on base_link.
    # You should replace this with actual transforms for where your aiding sensors are
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "1.52",
          "--y", "-0.36",
          "--z", "1.5",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "umrr"
        ]
    ),

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])
