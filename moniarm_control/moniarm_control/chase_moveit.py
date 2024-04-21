#!/usr/bin/env python

"""
Node for control 3DOF robot arm from joint_states
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from sensor_msgs.msg import JointState
from .submodules.myutil import Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *
from std_msgs.msg import Int32, Int32MultiArray

class ChaseMoveit(Node):
    def __init__(self):
        super().__init__('moveit_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
           ])
        self.get_logger().info("Setting Up the Node...")

        self.robotarm = Moniarm()
        self.robotarm.home()

        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.moveit_callback, 10)
        self.get_logger().info("Moveit Subscriber Awaked!! Waiting for Moveit Planning...")

    def moveit_callback(self, cmd_msg):
        motorMsg = Int32MultiArray()
        motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF]

        #print( str(cmd_msg.position[0]) + ':' + str(cmd_msg.position[1]) + ':' + str(cmd_msg.position[2]) + ':' + str(cmd_msg.position[3]) )

        motorMsg.data[0] = trimLimits(radiansToDegrees(cmd_msg.position[0]))
        motorMsg.data[1] = trimLimits(radiansToDegrees(cmd_msg.position[1]))
        motorMsg.data[2] = trimLimits(radiansToDegrees(cmd_msg.position[2]))
        motorMsg.data[3] = trimLimits(radiansToDegrees(cmd_msg.position[3]))
        self.robotarm.run(motorMsg)

        self.timediff = time() - self.prev_time
        self.prev_time = time()
        print( str(motorMsg.data[0]) + ':' + str(motorMsg.data[1]) + ':' + str(motorMsg.data[2])
        + ':' + str(motorMsg.data[3]) + ':' + str(self.timediff))

    def __del__(self):
        print("Arm class release")
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    myMoveit = ChaseMoveit()
    rclpy.spin(myMoveit)

    myMoveit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
