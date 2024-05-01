#!/usr/bin/env python3
#
# Copyright (c) 2024, ChangWhan Lee
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from time import sleep, time
import sys
import rclpy
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from std_msgs.msg import Int32, Int32MultiArray
from rclpy.node import Node
import os
from .submodules.myconfig import *

msg = """
Mimic Human's operation!
Caution: need to run teleop first
"""

def main():
    rclpy.init()

    node = rclpy.create_node('mimic_teleop_node')        # generate node

    robotarm = Moniarm()
    robotarm.home()

    print('moniarm mimic human operation')

    try:
        print(msg)
        rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_control/moniarm_control/')
        moveHistory = open(rosPath + 'automove.txt', 'r')

        motorMsg = Int32MultiArray()
        motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME]

        while(1):
            # Get next line from file
            line = moveHistory.readline()
            # if line is empty, end of file is reached
            if not line:
                break

            motor0, motor1, motor2, motor3, time_diff = line.split(':')
            motorMsg.data[0] = int(motor0)
            motorMsg.data[1] = int(motor1)
            motorMsg.data[2] = int(motor2)
            motorMsg.data[3] = int(motor3)

            try:
                sleep(float(time_diff))
            except KeyboardInterrupt:
                break

            robotarm.run(motorMsg)
            sys.stdout.write(str(motor0) + ':' + str(motor1)+':' + str(motor2) + ':' + str(motor3) + ':' + str(time_diff))
            sys.stdout.flush()

    except Exception as e:
        print(e)

    finally:  #
        #motor parking angle
        robotarm.park()

if __name__ == '__main__':
    main()
