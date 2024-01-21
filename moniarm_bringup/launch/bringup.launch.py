#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("moniarm_bringup"), '/launch', '/mcu.launch.py'])
    ),
  ])
