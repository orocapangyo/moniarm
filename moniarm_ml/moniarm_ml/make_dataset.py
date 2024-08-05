#!/usr/bin/env python3
#
# Copyright (c) 2011, Willow Garage, Inc.
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
# https://velog.io/@dumok_/ROS2-%EC%84%9C%EB%B9%84%EC%8A%A4-%EB%A1%9C%EB%B4%87%EC%97%90-TTS-%EC%A0%81%EC%9A%A9%ED%95%98%EA%B8%B0

from time import sleep, time
import os
import select
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped

from moniarm_interfaces.srv import Init
from moniarm_interfaces.msg import CmdMotor
from .submodules.myutil import Moniarm, setArmAgles
from .submodules.myconfig import *

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control Your Robot!
---------------------------
press space to make dataset
CTRL-C to quit
"""

e = """
Communications Failed
"""

class ServicenSubscriber(Node):
    def __init__(self):
        super().__init__('ClientAsyncInit')

        qos_profile1 = qos_profile_sensor_data
        self.blob_subscriber = self.create_subscription(
            CmdMotor,
            "/angle_motor",
            self.update_angle,
            qos_profile1
        )

        qos_profile2 = qos_profile_sensor_data
        self.blob_subscriber = self.create_subscription(
            PointStamped,
            "/blob/point_blob",
            self.update_ball,
            qos_profile2
        )

        self.cli = self.create_client(Init, 'Init')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Init service not available, waiting again...')
        self.req = Init.Request()

    def update_ball(self, message):
        #ignore 1 second previous message
        msg_secs = message.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if (msg_secs + 1 < now):
            #self.get_logger().info("Stamp %d, %d" %(now, msg_secs ) )
            return

        self.blob_x = message.point.x
        self.blob_y = message.point.y
        self.get_logger().info("Detected blob: %.2f  %.2f "%(self.blob_x, self.blob_y))
        return

    def update_angle(self, message):
        self.angle0 = message.angle0
        self.angle1 = message.angle1
        self.angle2 = message.angle2
        self.angle3 = message.angle3
        self.get_logger().info("Detected angle: %d  %d %d %d "%(self.angle0, self.angle1, self.angle2, self.angle3))
        return

    def send_request(self, a):
        self.req.motor_mode = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:  # if valid, save to key
        key = sys.stdin.read(1)
    else:       # else, initialize
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG, MAX_ANG)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_dataset_node')        # generate node

    robotarm = Moniarm()
    robotarm.home()
    print('moniarm IK dataset collector')

    # just key check
    status = 0
    svcSubscriber = ServicenSubscriber()

    #torqu on, normal status
    mStatus = 0
    blob_x = 0.0
    blob_y = 0.0
    angle0 = 0
    angle1 = 0
    angle2 = 0
    angle3 = 0

    rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_ml/moniarm_ml/')
    fhandle = open(rosPath + 'kinematics_pose.csv', 'w')
    fhandle.write('x+1,y+1,angle0,angle1,angle2,angle3\n')
    motorMsg = CmdMotor()
    #M0, M3 torque off by default
    setArmAgles(motorMsg, MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN)

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == ' ':
                if mStatus == 0:
                    print('torque off')
                    svcSubscriber.send_request(1)
                    blob_x = svcSubscriber.blob_x
                    blob_y = svcSubscriber.blob_y
                    mStatus = 1
                else:
                    print('torque on')
                    svcSubscriber.send_request(2)
                    angle0 = svcSubscriber.angle0
                    angle1 = svcSubscriber.angle1
                    angle2 = svcSubscriber.angle2
                    angle3 = svcSubscriber.angle3
                    mStatus = 0

                status = status + 1

            else:
                #Ctrl-C, then stop working
                if (key == '\x03'):
                    break
                #no valid input, then continue to read key
                else:
                    continue

            #key pressed, print motor angle
            if status == 1:
                if mStatus == 0:
                    print('x= %.2f, y=%.2f, M0= %d, M1=%d, M2=%d, M3=%d' %(blob_x, blob_y, angle0, angle1, angle2, angle3))
                    fhandle.write(str(blob_x) + ',' + str(blob_y) + ',' + str(angle0) + ',' + str(angle1) + ',' + str(angle2)
                                + ',' + str(angle3) + '\n')
                    #move home to check collect more data
                    robotarm.home()
                status = 0

    except Exception as e:
        print(e)

    finally:
        #motor parking angle
        print('Arm parking, be careful')
        robotarm.park()

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
