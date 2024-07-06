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
Node for control 3DOF robot arm from joint_states

Subscribes to
    /joint_states
Publishes commands to
    /cmd_motor

"""

from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from sensor_msgs.msg import JointState
import atexit
from rclpy.qos import qos_profile_sensor_data

from moniarm_interfaces.msg import CmdMotor
from .submodules.myutil import Moniarm, trimLimits, radiansToDegrees, setArmAgles
from .submodules.myconfig import *

class ChaseMoveit(Node):
    def __init__(self):
        super().__init__('state_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
           ])
        self.get_logger().info("Setting Up the Node...")

        self.robotarm = Moniarm()
        #can't move direct to zero, so home position first
        self.robotarm.home()
        self.robotarm.zero()

        self.motorMsg = CmdMotor()
        setArmAgles(self.motorMsg, MOTOR0_ZERO, MOTOR1_ZERO, MOTOR2_ZERO, MOTOR3_ZERO, GRIPPER_OPEN, 0.0)
        atexit.register(self.set_park)
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.moveit_callback, qos_profile_sensor_data)
        self.get_logger().info("Moveit Subscriber Awaked!! Waiting for Moveit Planning...")

    def moveit_callback(self, cmd_msg):
        self.motorMsg.angle0 = trimLimits(radiansToDegrees(cmd_msg.position[0]))
        self.motorMsg.angle1 = trimLimits(radiansToDegrees(cmd_msg.position[1]))
        self.motorMsg.angle2 = trimLimits(radiansToDegrees(cmd_msg.position[2]))
        self.motorMsg.angle3 = trimLimits(radiansToDegrees(cmd_msg.position[3]))
        #can't control air pump, then grip=0 always
        setArmAgles(self.motorMsg, self.motorMsg.angle0 , self.motorMsg.angle1 , self.motorMsg.angle2 , self.motorMsg.angle3, 0, 0.0)

        self.robotarm.run(self.motorMsg)
        print( str(self.motorMsg.angle0) + ':' + str(self.motorMsg.angle1) + ':' + str(self.motorMsg.angle2) + ':' + str(self.motorMsg.angle3) )

    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    myMoveit = ChaseMoveit()
    rclpy.spin(myMoveit)

    myMoveit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
