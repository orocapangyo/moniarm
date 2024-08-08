#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  motor_parameter = LaunchConfiguration(
    'motor_parameter',
    default=os.path.join(
      get_package_share_directory('moniarm_control'),
      'param/motor.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('motor_parameter', default_value=motor_parameter),

    Node(
      package='moniarm_control', executable='motor_chase', name='lowlevel_control_node',
	    #output='screen', emulate_tty=True,
      emulate_tty=True,
      parameters=[motor_parameter],
    ),
  ])
