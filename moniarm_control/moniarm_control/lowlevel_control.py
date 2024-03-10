#!/usr/bin/env python

"""
referenced from those projects

DkLowLevelCtrl, ServoConvert part from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

"""
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from geometry_msgs.msg import Twist
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from std_msgs.msg import Int32, Int32MultiArray

class ServoConvert:
    def __init__(self, id=1, center_value=0, range=180, direction=1):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range # 45
        self._dir = direction # 1 or -1
        self.id = id

        # --- Convert its range in [-1, 1]
        self._sf = 1.0 / self._half_range # 1 / 45

    def get_value_out(self, value_in):
        # --- twist type value is in  [-1, 1]
        self.value = value_in
        self.value_out = int(self._dir * value_in * self._half_range + self._center)
        return self.value_out

class LowLevelCtrl(Node):
    def __init__(self):
        super().__init__('lowlevel_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
           ])
        self.get_logger().info("Setting Up low level arm control node...")

        # Create a timer that will gate the node actions twice a second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.robotarm = Moniarm()
        self.robotarm.home()

        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [0, 0, 0, 0]
        self.get_logger().info("Setting Up low level arm control node...")

        self.SPEED_CENTER = 0
        self.SPEED_LIMIT = 180
        self.STEER_CENTER = 0
        self.STEER_LIMIT = 180

        self.actuators = {}
        self.actuators["throttle"] = ServoConvert(id=1, center_value=self.SPEED_CENTER, range=self.SPEED_LIMIT*2, direction=1)
        self.actuators["steering"] = ServoConvert(id=2, center_value=self.STEER_CENTER, range=self.STEER_LIMIT*2, direction=1)
        self.get_logger().info("> Actuators corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_twist = self.create_subscription(Twist, "/dkcar/control/cmd_vel", self.update_message_from_chase, 10)
        self.get_logger().info("> Subscriber corrrectly initialized")

        self.throttle_cmd = 0.0
        self.throttle_chase = 0.0
        self.steer_cmd = 0.0
        self.steer_chase = 0.0

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._last_time_chase_rcv = time.time()
        self._timeout_ctrl = 100
        self._timeout_blob = 1

        self.get_logger().info("Initialization complete")

    def update_message_from_command(self, message):
        self._last_time_cmd_rcv = time.time()
        self.throttle_cmd = message.linear.x
        self.steer_cmd = message.angular.z

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time.time()
        self.throttle_chase = message.linear.x
        self.steer_chase = message.angular.z
        #print(self.throttle_chase, self.steer_chase)

    def compose_command_velocity(self):
        self.throttle = clamp(self.throttle_cmd + self.throttle_chase, -1, 1)
        # -- Add steering
        self.steer = clamp(self.steer_cmd + self.steer_chase, -1, 1)
        self.set_actuators_from_cmdvel(self.throttle, self.steer)

    def set_actuators_from_cmdvel(self, throttle, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Convert vel into servo values
        self.actuators["throttle"].get_value_out(throttle)
        self.actuators["steering"].get_value_out(steering)
        # self.get_logger().info("Got a command v = %2.1f  s = %2.1f"%(throttle, steering))

        self.set_pwm_pulse(self.actuators["throttle"].value_out, self.actuators["steering"].value_out)
        print( "throttle: " + str(self.actuators["throttle"].value_out) +  ", steering: " + str(self.actuators["steering"].value_out))


    def set_pwm_pulse(self, speed_pulse, steering_pulse):
        self.motorMsg.data[0] = steering_pulse
        self.motorMsg.data[0] = speed_pulse
        self.robotarm.run(self.motorMsg)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.throttle_cmd = 0.0
        self.steer_cmd = 0.0

    def reset_avoid(self):
        self.throttle_chase = 0.0
        self.steer_avoid = 0.0

    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time.time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time.time() - self._last_time_chase_rcv < self._timeout_blob

    def node_callback(self):

        self.compose_command_velocity()
        if not self.is_controller_connected:
            self.set_actuators_idle()

        if not self.is_chase_connected:
            self.reset_avoid()

def main(args=None):
    rclpy.init(args=args)
    arm_llc = LowLevelCtrl()
    rclpy.spin(arm_llc)

    arm_llc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
