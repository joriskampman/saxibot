"""
A launch script for the following:
  1. starting the robot state publisher (rsp, included from other launch file)
  2. start Gazebo
  3. spawn saxibot

author: Joris Kampman, Saxion SMART, july 2024
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
  """ 
  Generate a LaunchDescription object
  """
  # Include the robot_state_publisher launch file, provided by our own package.
  package_name='saxibot'
  # Force sim time to be enabled

  # launcher for ROBOT STATE PUBLISHER
  # note 'use_sim_time' is true (todo: parameterize)
  rsp_launch_file = os.path.join(get_package_share_directory(package_name),
                                 'launch',
                                 'rsp.launch.py')
  rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource([rsp_launch_file]),
                                 launch_arguments={'use_sim_time': 'false'}.items())

  camera_launch_file = os.path.join(get_package_share_directory(package_name),
                                    'launch',
                                    'camera.launch.py')
  camera = IncludeLaunchDescription(PythonLaunchDescriptionSource([camera_launch_file]))

  gamepad_launch_file = os.path.join(get_package_share_directory(package_name),
                                     'launch',
                                     'joystick.launch.py')
  gamepad = IncludeLaunchDescription(PythonLaunchDescriptionSource([gamepad_launch_file]))

  robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
  controller_params_file = os.path.join(get_package_share_directory(package_name),
                                        'config',
                                        'my_controllers.yaml')


  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[{'robot_description': robot_description},
                controller_params_file],
  )
  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
  )

  joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
  )

  # add delays to controller manager
  delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
  delayed_diff_drive_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(target_action=controller_manager,
                                 on_start=[diff_drive_spawner])
  )
  
  delayed_joint_broad_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(target_action=controller_manager,
                                 on_start=[joint_broad_spawner])
  )

  # add lidar
  lidar_launch_file = os.path.join(get_package_share_directory('rplidar_ros'), 
                                  'launch',
                                  'rplidar_c1_launch.py')
  lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource([lidar_launch_file]))

  # Launch them all
  return LaunchDescription([rsp,
                            delayed_controller_manager,
                            delayed_diff_drive_spawner,
                            delayed_joint_broad_spawner,
                            camera,
                            lidar,
                            gamepad,
                            ])
