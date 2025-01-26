<<<<<<< HEAD
# pylint: disable=C0114,C0116
=======
>>>>>>> 8dfc5c0d7a9da5742bd655b1c45715c0cc800bc5
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  joy_params = os.path.join(get_package_share_directory('saxibot'),
                            'config',
                            'joystick.yaml')

<<<<<<< HEAD
  joy_node = Node(package='joy',
                  executable='joy_node',
                  parameters=[joy_params])

  teleop_node = Node(package='teleop_twist_joy',
                  executable='teleop_node',
                  name='teleop_node',
                  parameters=[joy_params],
                  remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')])

  return LaunchDescription([
    joy_node,
=======
  # joy_node = Node(package='joy',
  #                 executable='joy_node',
  #                 parameters=[joy_params])

  teleop_node = Node(package='teleop_twist_joy',
                     executable='teleop_node',
                     name='teleop_node',
                     parameters=[joy_params],
                     remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')])

  return LaunchDescription([
    # joy_node,
>>>>>>> 8dfc5c0d7a9da5742bd655b1c45715c0cc800bc5
    teleop_node,
  ])
