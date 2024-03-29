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
from geometry_msgs.msg import Twist, Point
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *

class ChaseBall(Node):
    def __init__(self):

        super().__init__('chase_ball_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('k_steer', 2.5),
           ])
        self.get_logger().info("Setting Up the Node...")
        self.K_LAT_DIST_TO_STEER = self.get_parameter_or('k_steer').get_parameter_value().double_value
        print('k_steer: %s' %
            (self.K_LAT_DIST_TO_STEER),
        )

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = self.create_subscription(Point, "/blob/point_blob", self.update_ball, 10)
        self.get_logger().info("Subscriber set")

        self.pub_twist = self.create_publisher(Twist, "/dkcar/control/cmd_vel", 10)
        self.get_logger().info("Publisher set")

        self._message = Twist()

        self._time_steer = 0
        self._steer_sign_prev = 0

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
        self.get_logger().info("Ball detected: %.2f  %.2f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action = 0.0
        object_detect = 0.0
        final_steer_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            blobx_diff = self.blob_x - 0.5
            if ((blobx_diff > IN_RANGE_MIN) and (blobx_diff < IN_RANGE_MAX)) :
                final_steer_action = 0.0
                self._message.angular.y = 1.0
            else:
                steer_action = DIR_TO_STEER * blobx_diff
                final_steer_action = steer_action*self.K_LAT_DIST_TO_STEER
                final_steer_action = clamp(final_steer_action, -1.0, 1.0)
                self._message.angular.y = 0.0

            object_detect = 1.0
            #if object is detected, go forward with defined power
            self.get_logger().info("Steering = %.2f" % (final_steer_action))

        return (object_detect, final_steer_action)

    def node_callback(self):
        # -- Get the control action
        object_detect, steer_action = self.get_control_action()
        #self.get_logger().info("RUN, Steering = %3.1f Detected = %3.1f" % (steer_action, object_detect))

        # -- update the message
        self._message.linear.x = object_detect
        self._message.angular.z = steer_action

        # -- publish it, only blob detected
        if self.is_detected:
            #self.get_logger().info("Steering = %.2f, object_detect = %.2f" %(steer_action, object_detect))
            self.pub_twist.publish(self._message)


def main(args=None):
    rclpy.init(args=args)
    chase_ball = ChaseBall()
    rclpy.spin(chase_ball)

    chase_ball.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
