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
        FindPackageShare("moniarm_cv"), '/launch', '/csicam.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("moniarm_control"), '/launch', '/chase_yolo.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("moniarm_control"), '/launch', '/lowlevel_ctl.launch.py'])
    ),

  ])

