#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
  cv_parameter = LaunchConfiguration(
    'cv_parameter',
    default=os.path.join(
      get_package_share_directory('moniarm_cv'),
      'param/cvparam.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('cv_parameter', default_value=cv_parameter),
    Node(
      package='moniarm_cv', executable='find_ball', name='blob_detect_node',
      output='screen', emulate_tty=True,
      parameters=[cv_parameter],
    ),
  ])
