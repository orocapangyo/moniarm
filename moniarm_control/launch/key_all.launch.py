#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("moniarm_bringup"), '/launch', '/mcu.launch.py'])
    ),

    Node(
      package='moniarm_teleop', executable='teleop_keyboard', name='teleop_keyboard_node',
      output='screen', emulate_tty=True,
      prefix='gnome-terminal --'
    ),

  ])
