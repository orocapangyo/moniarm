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
from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from darknet_ros_msgs.msg import BoundingBoxes
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *
from moniarm_interfaces.msg import CmdChase

class ChaseObject(Node):
    def __init__(self):

        super().__init__('chase_object_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('K_x', 2.5),
                ('DETECT_CLASS1', "RedStar"),
                ('DETECT_CLASS2', "BlueCylinder"),
           ])
        self.get_logger().info("Setting Up the Node...")
        self.K_x = self.get_parameter_or('K_x').get_parameter_value().double_value
        self.DETECT_CLASS1 = self.get_parameter_or('DETECT_CLASS1').get_parameter_value().string_value
        self.DETECT_CLASS2 = self.get_parameter_or('DETECT_CLASS2').get_parameter_value().string_value

        print('K_x: %s, DETECT_CLASS 1: %s, DETECT_CLASS 2: %s'%
            (self.K_x,
            self.DETECT_CLASS1,
            self.DETECT_CLASS2)
        )

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.sub_center = self.create_subscription(BoundingBoxes, "/darknet_ros/bounding_boxes", self.update_object, 10)
        self.get_logger().info("Subscriber set")

        self.pub_chase = self.create_publisher(CmdChase, "/control/cmd_chase", 10)
        self.get_logger().info("Publisher set")

        self._message = CmdChase()

        # Create a timer that will gate the node actions twice a second
        timer_period = Ktimer
        self.timer = self.create_timer(timer_period, self.node_callback)

    @property
    def is_detected(self):
        return time() - self._time_detected < 1.0

    def update_object(self, message):
        for box in message.bounding_boxes:
            #
            #yolov4-tiny, 416x416
            if (box.class_id == self.DETECT_CLASS1) or (box.class_id == self.DETECT_CLASS2):
                self.blob_x = float((box.xmax + box.xmin)/PICTURE_YOLO/2.0) - 0.5
                self.blob_y = float((box.ymax + box.ymin)/PICTURE_YOLO/2.0) - 0.5
                self._time_detected = time()

                if box.class_id == self.DETECT_CLASS1:
                    self.detect_object = 1
                else:
                    self.detect_object = 2
                self.get_logger().info("object detected: %.2f  %.2f "%(self.blob_x, self.blob_y))
            else:
                self.detect_object = 0

    def get_control_action(self, object):
        """
        Based on the current ranges, calculate the command
        """
        command_x = 0.0
        inrange = 0
        detect_object = object

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            command_x = self.K_x * self.blob_x
            command_x = clamp(command_x, -1.0, 1.0)
            if ((command_x > IN_RANGE_MIN) and (command_x < IN_RANGE_MAX)) :
                inrange = 1
            #if object is detected, go forward with defined power
            #self.get_logger().info("CommandX= %.2f" % (command_x))

        return (detect_object, command_x, inrange)

    def node_callback(self):
         # -- update the message
        self._message.object, self._message.cmd_x, self._message.inrange = self.get_control_action(self.detect_object)
        self._message.stamp = self.get_clock().now().to_msg()
        
        # -- publish it, only blob detected
        if self.is_detected:
            self.get_logger().info("CommandX= %.3f In= %d " %(self._message.cmd_x, self._message.inrange) )
            self.pub_chase.publish(self._message)
        #else:
        #    self.get_logger().info("Missing object")

def main(args=None):
    rclpy.init(args=args)
    chase_object = ChaseObject()
    rclpy.spin(chase_object)

    chase_object.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
