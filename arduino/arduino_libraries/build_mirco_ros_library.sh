#!/bin/sh

ROS_PKG=galactic
MONIARM_SRC=~/ros2_ws/src/moniarm/

git clone -b $ROS_PKG https://github.com/micro-ROS/micro_ros_arduino
cd micro_ros_arduino/extras/library_generation/extra_packages

cp -r $MONIARM_SRC/moniarm_interfaces  ./micro_ros_arduino/extras/library_generation/extra_packages

ros2 pkg create --build-type ament_cmake uros_moniarm_interfaces

cd micro_ros_arduino

docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:$ROS_PKG -p esp32
