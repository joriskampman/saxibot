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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
  """ 
  Generate a LaunchDescription object
  """
  # Include the robot_state_publisher launch file, provided by our own package.
  # Force sim time to be enabled
  package_name='saxibot'

  rsp = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory(package_name),'launch','rsp.launch.py'
              )]), launch_arguments={'use_sim_time': 'true'}.items()
  )

  # Include the Gazebo launch file, provided by the gazebo_ros package
  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
           )

  # Run the spawner node from the gazebo_ros package.
  # The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', 'saxibot'],
                      output='screen')



  # Launch them all!
  return LaunchDescription([rsp,
                            gazebo,
                            spawn_entity])
