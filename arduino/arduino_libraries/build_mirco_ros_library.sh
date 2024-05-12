#!/bin/sh

# change env for your ros package
ROS_PKG=galactic
MONIARM_SRC=~/ros2_ws/src/moniarm
ARDUINO_ROOT=~/Arduino

# 1. download micro_ros_arduino source code from github
git clone -b $ROS_PKG https://github.com/micro-ROS/micro_ros_arduino

# 2. copy moniarm custom message, service interface package(moniarm_interfaces) from the Moniarm main interface directory to mirco_ros extra_packages directory.
cp -rf $MONIARM_SRC/moniarm_interfaces  ./micro_ros_arduino/extras/library_generation/extra_packages/

# 3. build micro_ros_arduino with docker
cd micro_ros_arduino
sed -i "s/-DRMW_UXRCE_MAX_SERVICES=1/-DRMW_UXRCE_MAX_SERVICES=5/g" extras/library_generation/colcon.meta
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:$ROS_PKG -p esp32

# 4.1 zip micro_ros_arduino for Arduino IDE libray
cd ..
zip -r micro_ros_arduino.zip micro_ros_arduino/*

# 4.2 copy micro_ros_arduino to Arduino libary
# cp -r micro_ros_arduino  $ARDUINO_ROOT/libraries/
