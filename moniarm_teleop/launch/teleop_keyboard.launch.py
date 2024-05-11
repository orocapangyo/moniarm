#!/usr/bin/env python3
# Author: Kyuhyong You
# Author: ChangWhan Lee
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  teleop_keyboard_parameter = LaunchConfiguration(
    'teleop_keyboard_parameter',
    default=os.path.join(
      get_package_share_directory('moniarm_teleop'),
      'param/teleop_keyboard.yaml'
    )
  )
  return LaunchDescription([
    DeclareLaunchArgument('teleop_keyboard_parameter', default_value=teleop_keyboard_parameter),

    Node(
      package='moniarm_teleop', executable='teleop_keyboard', name='teleop_keyboard_node',
      output='screen',
      emulate_tty=True,
      parameters=[teleop_keyboard_parameter],
    ),

  ])