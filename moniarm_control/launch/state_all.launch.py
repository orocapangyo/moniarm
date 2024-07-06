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
        FindPackageShare("moniarm_description"), '/launch', '/moniarm_description.launch.py'])
    ),

    Node(
      package='moniarm_control', executable='chase_moveit', name='state_control_node',
      output='screen', emulate_tty=True,
    ),
  ])
