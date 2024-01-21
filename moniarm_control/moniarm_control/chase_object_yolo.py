#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to
    /darknet_ros/bounding_boxes
Publishes commands to
    /dkcar/control/cmd_vel

"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from rclpy.logging import get_logger

PICTURE_SIZE = 416.0

def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value

class ChaseObject(Node):
    def __init__(self):

        super().__init__('chase_object_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('k_steer', 2.5),
                ('k_throttle', 0.2),
                ('DETECT_CLASS', "cup"),
           ])
        self.get_logger().info("Setting Up the Node...")

        self.K_LAT_DIST_TO_STEER = self.get_parameter_or('k_steer').get_parameter_value().double_value
        self.K_LAT_DIST_TO_THROTTLE = self.get_parameter_or('k_throttle').get_parameter_value().double_value
        self.DETECT_CLASS = self.get_parameter_or('DETECT_CLASS').get_parameter_value().string_value

        print('k_steer: %s, k_throttle: %s, DETECT_CLASS: %s'%
            (self.K_LAT_DIST_TO_STEER,
            self.K_LAT_DIST_TO_THROTTLE,
            self.DETECT_CLASS)
        )

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = self.create_subscription(BoundingBoxes, "/darknet_ros/bounding_boxes", self.update_object, 10)
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

    def update_object(self, message):
        for box in message.bounding_boxes:
            #
            #yolov4-tiny, 416x416
            if box.class_id == self.DETECT_CLASS:
                self.blob_x = float((box.xmax + box.xmin)/PICTURE_SIZE/2.0) - 0.5
                self.blob_y = float((box.ymax + box.ymin)/PICTURE_SIZE/2.0) - 0.5
                self._time_detected = time.time()
                self.get_logger().info("object detected: %.2f  %.2f "%(self.blob_x, self.blob_y))
                #self.get_logger().info(
                #    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {} Class: {}".format
                #    (box.xmin, box.xmax, box.ymin, box.ymax, box.Class) )
    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action = 0.0
        throttle_action = 0.0

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = self.K_LAT_DIST_TO_STEER * self.blob_x
            steer_action = saturate(steer_action, -1.5, 1.5)
            self.get_logger().info("BlobX %.2f" % self.blob_x)

            #if object is detected, go forward with defined power
            throttle_action = self.K_LAT_DIST_TO_THROTTLE
            self.get_logger().info("is _detected, Steering = %3.1f Throttle = %3.1f" % (steer_action, throttle_action))

        return (steer_action, throttle_action)

    def node_callback(self):
        # -- Get the control action
        steer_action, throttle_action = self.get_control_action()
        self.get_logger().info("RUN, Steering = %3.1f Throttle = %3.1f" % (steer_action, throttle_action))

        # -- update the message
        self._message.linear.x = throttle_action
        self._message.angular.z = steer_action

        # -- publish it
        self.pub_twist.publish(self._message)

def main(args=None):
    rclpy.init(args=args)
    chase_object = ChaseObject()
    rclpy.spin(chase_object)

    chase_object.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
