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
    /darknet_ros/bounding_boxes
Publishes commands to
    /control/cmd_chase

"""
from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from darknet_ros_msgs.msg import BoundingBoxes
from rclpy.qos import qos_profile_sensor_data
from moniarm_interfaces.msg import CmdMotor
import atexit

from .submodules.myutil import Moniarm, setArmAgles
from .submodules.myconfig import *

class ChaseObject(Node):
    def __init__(self):

        super().__init__('chase_object_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('DETECT_CLASS1', "watermelon"),
                ('DETECT_CLASS2', "pineapple"),
                ('K_a', 0.0),
                ('K_b', 0.0),
           ])
        
        self.get_logger().info("Setting Up the Node...")
        self.DETECT_CLASS1 = self.get_parameter_or('DETECT_CLASS1').get_parameter_value().string_value
        self.DETECT_CLASS2 = self.get_parameter_or('DETECT_CLASS2').get_parameter_value().string_value
        print('DETECT_CLASS 1: %s, DETECT_CLASS 2: %s'%
            (self.DETECT_CLASS1,
            self.DETECT_CLASS2)
        )
        self.K_a = self.get_parameter_or('K_a').get_parameter_value().double_value
        self.K_b = self.get_parameter_or('K_b').get_parameter_value().double_value
        print('K_a, K_b: %s %s' %
            (self.K_a, self.K_b)
        )

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.sub_center = self.create_subscription(BoundingBoxes, "/darknet_ros/bounding_boxes", self.update_object, qos_profile_sensor_data)
        self.get_logger().info("Subscriber set")

         # Create a timer that will gate the node actions twice a second
        self.timer = self.create_timer(Ktimer, self.node_callback)

        self.motorMsg = CmdMotor()
        setArmAgles(self.motorMsg, MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN)
        self.robotarm = Moniarm()
        self.armStatus = 'HOMING'
        self.robotarm.home()
        self.armStatus = 'SEARCHING'

    @property
    def is_detected(self):
        return time() - self._time_detected < 1.0

    def update_object(self, message):
        #ignore 1 second previous message
        msg_secs = message.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if (msg_secs + 1 < now):
            self.get_logger().info("Stamp %d, %d" %(now, msg_secs ) )
            return
        
        for box in message.bounding_boxes:
            #
            #yolov4-tiny, 416x416
            if (box.class_id == self.DETECT_CLASS1) or (box.class_id == self.DETECT_CLASS2):
                self.blob_x = float((box.xmax + box.xmin)/PICTURE_SIZE_X) - 1.0
                self.blob_y = float((box.ymax + box.ymin)/PICTURE_SIZE_Y) - 1.0
                self._time_detected = time()

                if box.class_id == self.DETECT_CLASS1:
                    self.detect_object = 1
                else:
                    self.detect_object = 2

                self.get_logger().info("Detected: %.2f  %.2f "%(self.blob_x, self.blob_y))
            else:
                self.detect_object = 0

    def get_control_action(self):
        if self.armStatus != 'SEARCHING' :
            print(self.armStatus)
            return

        if self.is_detected == 1:
            self.armStatus = 'PICKUP'
            #caculate angles from linear equation
            input_ = self.blob_x + 1.0
            outputx = self.K_a*(self.blob_x + 1.0) + self.K_b
            print(f"input: {input_}")
            print(f"output: {outputx}")

            #motor move directly
            self.get_logger().info("Go to object")
            self.motorMsg.angle0 = int(outputx)
            self.motorMsg.angle1 = MOTOR_NOMOVE
            self.motorMsg.angle2 = MOTOR_NOMOVE
            self.motorMsg.angle3 = MOTOR_NOMOVE
            self.robotarm.run(self.motorMsg)
            sleep(1.0)

            self.get_logger().info("Picking up")
            #then pick it up, need new function
            self.robotarm.picknplace(self.detect_object, 1)
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
    chase_object = ChaseObject()
    rclpy.spin(chase_object)

    chase_object.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
