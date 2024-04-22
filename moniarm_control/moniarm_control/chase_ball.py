#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to
    /blob/point_blob
Publishes commands to
    /dkcar/control/cmd_vel

"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from geometry_msgs.msg import Point
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *
from moniarm_interfaces.msg import CmdChase

class ChaseBall(Node):
    def __init__(self):

        super().__init__('chase_ball_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('K_x', 2.5),
           ])
        self.get_logger().info("Setting Up the Node...")
        self.K_x = self.get_parameter_or('K_x').get_parameter_value().double_value
        print('K_x: %s' %
            (self.K_x),
        )

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = self.create_subscription(Point, "/blob/point_blob", self.update_ball, 10)
        self.get_logger().info("Subscriber set")

        self.pub_chase = self.create_publisher(CmdChase, "/control/cmd_chase", 10)
        self.get_logger().info("Publisher set")

        self._message = CmdChase()

        # Create a timer that will gate the node actions twice a second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        self.get_logger().info("Ball detected x, y: %.2f  %.2f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        command_x = 0.0
        detect_object = 0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            blobx_diff = self.blob_x
            if ((blobx_diff > IN_RANGE_MIN) and (blobx_diff < IN_RANGE_MAX)) :
                final_steer_action = 0.0
                self._message.inrange = 1
            else:
                command_x = DIR_TO_X * blobx_diff
                command_x = command_x*self.K_x
                command_x = clamp(command_x, -1.0, 1.0)
                self._message.inrange = 0

            detect_object = 1
            #if object is detected, go forward with defined power
            #self.get_logger().info("CommandX= %.2f" % (command_x))

        return (detect_object, command_x)

    def node_callback(self):
         # -- update the message
        self._message.object, self._message.cmd_x = self.get_control_action()

        # -- publish it, only blob detected
        if self.is_detected:
            self.get_logger().info("CommandX= %.2f" % (self._message.cmd_x))
            self.pub_chase.publish(self._message)


def main(args=None):
    rclpy.init(args=args)
    chase_ball = ChaseBall()
    rclpy.spin(chase_ball)

    chase_ball.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()