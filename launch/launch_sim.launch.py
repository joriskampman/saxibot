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
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


package_name='saxibot'

def generate_launch_description():
  """ 
  Generate a LaunchDescription object
  """
  # Include the robot_state_publisher launch file, provided by our own package.
  # Force sim time to be enabled
  package_dir = get_package_share_directory(package_name)
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  # ------------------- files and folders  ---------------------------------
  ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
  gz_launch_file = os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')

  gz_model_dir = os.path.join(package_dir, 'models')
  world_file = os.path.join(package_dir, 'worlds', 'empty_world.sdf')
  rsp_launch_file = os.path.join(package_dir, 'launch', 'rsp.launch.py')

  # -------------- actions to do --------------------------------------------------
  # add to (append) environment variab le for the gazebo resources with models directory
  append_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                                        gz_model_dir)

  # ---------- launch gazebo itself ------------------------
  # Include the Gazebo launch file, provided by the gazebo_ros package
  gz_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gz_launch_file),
    launch_arguments={'gz_args': ['-r -s -v4 ', world_file],
                      'on_exit_shutdown': 'true'}.items()
  )

  gz_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gz_launch_file),
    launch_arguments={'gz_args': '-g -v4 '}.items()
  )

  # ---- robot state publisher ---------------------------
  # takes use_sim_time from commandline, otherwise default
  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(rsp_launch_file),
    launch_arguments={'use_sim_time': use_sim_time}.items()
  )

  # Run the spawner node from the gazebo_ros package.
  # The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='ros_gz_sim',
                      executable='create',
                      arguments=['-topic', 'robot_description',
                                  '-name', 'saxibot',
                                  '-z', '1.0'],
                      output='screen')

  ld = LaunchDescription()

  ld.add_action(append_env_vars_resources)
  ld.add_action(gz_server_cmd)
  ld.add_action(gz_client_cmd)
  ld.add_action(rsp)
  ld.add_action(spawn_entity)
  # Launch them all!

  return ld
