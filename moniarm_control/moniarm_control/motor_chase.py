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
from std_msgs.msg import Int32, Int32MultiArray
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *
from moniarm_interfaces.msg import CmdChase

class ServoConvert:
    def __init__(self, id=1, center_value=0, range=MAX_LIN_VEL, direction=1):
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
        # --- type value is in  [-1, 1]
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

        # Create a timer that will gate the node actions twice a second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF]
        self.get_logger().info("Setting Up low level arm control node...")

        self.actuators = {}
        self.actuators["axis_x"] = ServoConvert(id=1, center_value=0, range=MAX_LIN_VEL, direction=1)
        self.get_logger().info("> actuator corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_chase = self.create_subscription(CmdChase, "/control/cmd_chase", self.update_message_from_chase, 10)
        self.get_logger().info("> Subscriber corrrectly initialized")

        self.command_x = 0.0
        self.chase_x_count = 0

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._last_time_chase_rcv = time.time()
        self._timeout_ctrl = 100
        self._timeout_command = 3
        self.detect_object = 0
        self.inrange = 0

        self.armStatus = "Homing"
        self.robotarm = Moniarm()
        self.robotarm.home()

        self.get_logger().info("Initialization complete")

    #don't use this function
    def update_message_from_command(self, message):
        self._last_time_cmd_rcv = time.time()
        self.command_x = message.cmd_x
        #self.get_logger().info("command: " +str(self.command_x))

    def update_message_from_chase(self, message):
        self._last_time_chase_rcv = time.time()
        self.detect_object = message.object
        self.command_x = message.cmd_x
        self.inrange = message.inrange
        #self.get_logger().info("chase: " + str(self.command_x))

    def compose_command_velocity(self):
        #if object is detected
        if (self.detect_object > 0):
            self.set_actuators_from_cmdvel(self.detect_object, self.command_x)

    def set_actuators_from_cmdvel(self, detect_object, command_x):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        if detect_object > 0:
            self.armStatus = "Searching"
            self.chase_x_count += 1
            #chase slowly
            if self.chase_x_count < 5:
                return
            else:
                self.chase_x_count = 0

        # steering is chase cmmand
        if self.inrange == 1:
            self.armStatus = "PickingUp"
            angle_x = int(self.actuators["axis_x"].get_value_out(command_x))
            self.set_angles(angle_x)
            self.get_logger().info("Object is inrange, then pick up. Object: %d" %(detect_object))
            self.robotarm.picknplace(detect_object)
            self.robotarm.home()

        # -- Convert vel into servo values
        angle_x=int(self.actuators["axis_x"].get_value_out(command_x))
        self.set_angles(angle_x)
        self.get_logger().info("Move Angle:%d, command: %2.2f"%(angle_x, command_x))

    def set_angles(self, angleX):
        self.motorMsg.data[0] = angleX
        self.motorMsg.data[1] = MOTOR_NOMOVE
        self.motorMsg.data[2] = MOTOR_NOMOVE
        self.motorMsg.data[3] = MOTOR_NOMOVE
        self.robotarm.run(self.motorMsg)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.command_x = 0.0
        self.detect_object = 0
        #self.get_logger().info("go to idle")

    def reset_avoid(self):
        self.detect_object = 0
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR_NOMOVE
        self.motorMsg.data[2] = MOTOR_NOMOVE
        self.motorMsg.data[3] = MOTOR_NOMOVE
        self.robotarm.run(self.motorMsg)
        #self.get_logger().info("reset avoid")
    @property
    def is_controller_connected(self):
        # print time.time() - self._last_time_cmd_rcv
        return time.time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time.time() - self._last_time_chase_rcv < self._timeout_command

    def node_callback(self):
        self.compose_command_velocity()
        if not self.is_controller_connected:
            self.set_actuators_idle()

        if not self.is_chase_connected:
            self.reset_avoid()

    def __del__(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    arm_llc = LowLevelCtrl()
    rclpy.spin(arm_llc)

    arm_llc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
