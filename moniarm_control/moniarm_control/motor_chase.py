#!/usr/bin/env python

"""
referenced from those projects

DkLowLevelCtrl, ServoConvert part from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

"""
from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from std_msgs.msg import Int32, Int32MultiArray
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from .submodules.myconfig import *
from moniarm_interfaces.msg import CmdChase

class ServoConvert:
    def __init__(self, id=1, center_value=0, range=MAX_ANG, direction=1):
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
        self.value_out = int(self._dir * value_in * self._half_range + self._center + 0.5)
        if self.value_out > 0:
            self.value_out = self.value_out + 180
        else:
            self.value_out = self.value_out - 180
        return self.value_out

class LowLevelCtrl(Node):
    def __init__(self):
        super().__init__('lowlevel_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
           ])

        # Create a timer that will gate the node actions twice a second
        timer_period = Ktimer
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF]
        self.get_logger().info("Setting Up low level arm control node...")

        self.actuators = {}
        self.actuators["axis_x"] = ServoConvert(id=1, center_value=0, range=MAX_ANG, direction=1)
        self.get_logger().info("> actuator corrrectly initialized")

        # --- Create the Subscriber to obstacle_avoidance commands
        self.ros_sub_chase = self.create_subscription(CmdChase, "/control/cmd_chase", self.update_message_from_chase, 10)
        self.get_logger().info("> Subscriber corrrectly initialized")

        self.command_x = 0.0
        self.command_x_prev = [0.0, 0.0, 0.0]
        self.detect_object = 0
        self.inrange = 0
        self.command_count = 0

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time()
        self._last_time_chase_rcv = time()
        self._timeout_ctrl = 100
        self._timeout_command = 2           #2 seconds

        self.armStatus = "Homing"
        self.get_logger().info("Homing")
        self.robotarm = Moniarm()
        self.robotarm.home()
        self.get_logger().info("Homing Done")
        self.armStatus = "Searching"

    #don't use this function
    def update_message_from_command(self, message):
        self._last_time_cmd_rcv = time()
        self.command_x = message.cmd_x
        #self.get_logger().info("command: " +str(self.command_x))

    def update_message_from_chase(self, message):
        if self.armStatus == "Searching":
            self._last_time_chase_rcv = time()
            self.detect_object = message.object
            self.command_x = message.cmd_x
            self.inrange = message.inrange
        else:
            self.detect_object = 0
            self.command_x = 0.0
            self.inrange = 0

    def compose_command_velocity(self):
        #if object is detected
        if (self.detect_object > 0):
            self.set_actuators_from_cmdvel(self.inrange, self.detect_object, self.command_x)
        else:
            self.command_x_prev = [0.0, 0.0, 0.0]

    def set_actuators_from_cmdvel(self, inrange, det_object, command):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        command_x = 0.0

        if self.armStatus == "PickingUp":
            self.get_logger().info("why here, PikingUp")
            return
        else:
            self.armStatus = "Searching"

        # steering is chase cmmand
        if inrange == 1:
            self.armStatus = "PickingUp"
            self.get_logger().info("Object is inrange, then pick up. Object: %d" %(det_object))
            self.robotarm.picknplace(det_object)
            self.get_logger().info("Now Home")
            self.armStatus = "Searching"
            self.reset_avoid()

        if self.command_count < 3:
            self.command_x_prev[self.command_count] = command
            self.get_logger().info( "command: %.3f, %.3f, %.3f" %(self.command_x_prev[0], self.command_x_prev[1],self.command_x_prev[2]) )
            self.command_count += 1
            return
        else:
            self.command_count = 0

        #simple PI control, Kp=1, Ki=0.3
        command_x = self.command_x_prev[0]*0.1 + self.command_x_prev[1]*0.2 + self.command_x_prev[2]*0.3+ command
        command_x = clamp(command_x, -1.00, 1.00)
        self.command_x_prev = [0.0, 0.0, 0.0]

        # -- Convert vel into servo values
        angle_x=self.actuators["axis_x"].get_value_out(command_x)
        self.set_angles(angle_x)
        self.get_logger().info("Move Angle:%d, command: %.3f"%(angle_x, command_x))

    def set_angles(self, angleX):
        self.motorMsg.data[0] = angleX
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = MOTOR3_HOME
        self.robotarm.run(self.motorMsg)

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.command_x = 0.0
        self.detect_object = 0
        #self.get_logger().info("go to idle")

    def reset_avoid(self):
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = MOTOR3_HOME
        self.robotarm.run(self.motorMsg)
        self.get_logger().info("reset avoid")

        self.detect_object = 0
        self.inrange = 0
        self.command_count = 0
        self.command_x_prev = [0.0, 0.0, 0.0]

    @property
    def is_controller_connected(self):
        # print time() - self._last_time_cmd_rcv
        return time() - self._last_time_cmd_rcv < self._timeout_ctrl

    @property
    def is_chase_connected(self):
        return time() - self._last_time_chase_rcv < self._timeout_command

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
