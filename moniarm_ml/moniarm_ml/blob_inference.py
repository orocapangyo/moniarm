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

"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to
    /blob/point_blob

"""
from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data
import atexit

from moniarm_interfaces.msg import  CmdMotor
from .submodules.myutil import Moniarm, setArmAgles
from .submodules.myconfig import *
from .iknet import IKNet

import os, torch

MAX_Y = 3

class IKnetBall(Node):
    def __init__(self):
        super().__init__('nn_blob_node')
        self.get_logger().info("Setting Up the Node...")

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.motorMsg = CmdMotor()
        #M0, M3 torque off by default
        setArmAgles(self.motorMsg, MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF, GRIPPER_OPEN)
        self.get_logger().info("Setting Up control node...")

        self.sub_center = self.create_subscription(PointStamped, "/blob/point_blob", self.update_ball, qos_profile_sensor_data)
        self.get_logger().info("Subscriber set")

        # Create a timer that will gate the node actions twice a second
        self.timer = self.create_timer(0.1, self.node_callback)

        self.robotarm = Moniarm()

        self.armStatus = 'HOMING'
        self.robotarm.home()
        self.armStatus = 'SEARCHING'

        atexit.register(self.set_park)

        rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_ml/moniarm_ml/')
        modely = rosPath + "iknet_y.pth"

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.modely = IKNet(MAX_Y)
        #print(self.modely)
        self.modely.to(self.device)
        self.modely.load_state_dict(torch.load(modely))
        self.modely.eval()

    @property
    def is_detected(self):
        return time() - self._time_detected < 1.0

    def update_ball(self, message):
        #ignore 1 second previous message
        msg_secs = message.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if (msg_secs + 1 < now):
            #self.get_logger().info("Stamp %d, %d" %(now, msg_secs ) )
            return
        self.blob_x = message.point.x
        self.blob_y = message.point.y
        self._time_detected = time()
        self.get_logger().info("Detected x, y: %.2f  %.2f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        if self.armStatus != 'SEARCHING' :
            print(self.armStatus)
            return

        if self.is_detected == 1:
            detect_object = 1           #blob detects only one object, then it's 1
            self.armStatus = 'PICKUP'
            #caculate angles from linear equation
            input_ = self.blob_x + 1.0
            outputx = K_a*(self.blob_x + 1.0) + K_b
            print(f"input: {input_}")
            print(f"output: {outputx}")

            #caculate angles from IKNet
            input_ = torch.FloatTensor([self.blob_y + 1.0])
            input_ = input_.to(self.device)
            print(f"input: {input_}")
            outputy = self.modely(input_)
            print(f"output: {outputy}")

            #motor move directly
            self.get_logger().info("Go to object")
            self.motorMsg.angle0 = int(outputx)
            self.motorMsg.angle1 = MOTOR_NOMOVE
            self.motorMsg.angle2 = MOTOR_NOMOVE
            self.motorMsg.angle3 = MOTOR_NOMOVE
            self.robotarm.run(self.motorMsg)
            sleep(1.0)
            self.motorMsg.angle0 = MOTOR_NOMOVE
            self.motorMsg.angle1 = MOTOR_NOMOVE
            self.motorMsg.angle2 = int(outputy[1].item()+0.5)
            self.motorMsg.angle3 = int(outputy[2].item()+0.5)
            self.robotarm.run(self.motorMsg)
            sleep(0.5)
            self.motorMsg.angle0 = MOTOR_NOMOVE
            #compenstate manually for far distance
            if self.blob_y < -0.45:
                self.motorMsg.angle1 = int(outputy[0].item()+4.5)
            elif self.blob_y < -0.25:
                self.motorMsg.angle1 = int(outputy[0].item()+2.5)
            elif self.blob_y < -0.10:
                self.motorMsg.angle1 = int(outputy[0].item()+1.5)
            elif self.blob_y < 0.10:
                self.motorMsg.angle1 = int(outputy[0].item()+0.5)
            else:
                self.motorMsg.angle1 = int(outputy[0].item()-0.5)

            self.motorMsg.angle2 = MOTOR_NOMOVE
            self.motorMsg.angle3 = MOTOR_NOMOVE
            self.robotarm.run(self.motorMsg)
            sleep(0.5)

            self.get_logger().info("Picking up")
            #then pick it up, need new function
            self.robotarm.picknplace(detect_object, 0)
            self.reset_avoid()

    def reset_avoid(self):
        self.motorMsg.angle0 = MOTOR0_HOME
        self.motorMsg.angle1 = MOTOR1_HOME
        self.motorMsg.angle2 = MOTOR2_HOME
        self.motorMsg.angle3 = MOTOR3_HOME
        self.robotarm.run(self.motorMsg)
        sleep(1.0)
        self.get_logger().info("reset avoid")
        self.armStatus = 'SEARCHING'
        self.detect_object = 0

    def node_callback(self):
         # -- update the message
        self.get_control_action()

    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    iknet_ball = IKnetBall()
    rclpy.spin(iknet_ball)
    iknet_ball.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
