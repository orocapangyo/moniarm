#!/usr/bin/env python3
#
# Copyright (c) 2011, Willow Garage, Inc.
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

from time import sleep, time
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from .submodules.myutil import Moniarm, clamp
from .submodules.myconfig import *
from std_msgs.msg import Int32MultiArray
from moniarm_interfaces.srv import SetLED, PlayAni, PlaySong

msg = """
Control Your Robot!
---------------------------
Moving around:
Left Stick left/right: Base(M0), left/light
Left Stick up/down:    shoulder(M1) move
Right Stick up/down:   Elbow(M2) move

'X' : gripper open/close
'A' : Change led
'B' : Play buzzer song
'Y' : Play OLED animation
"""

class ClientAsyncLed(Node):
    def __init__(self):
        super().__init__('LEDClientAsync')
        self.cli = self.create_client(SetLED, 'SetLED')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting again...')
        self.req = SetLED.Request()

    def send_request(self, a):
        self.req.index = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class ClientAsyncAni(Node):
    def __init__(self):
        super().__init__('ClientAsyncAni')
        self.cli = self.create_client(PlayAni, 'PlayAni')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('OLED service not available, waiting again...')
        self.req = PlayAni.Request()

    def send_request(self, a):
        self.req.index = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class ClientAsyncSong(Node):
    def __init__(self):
        super().__init__('ClientAsyncSong')
        self.cli = self.create_client(PlaySong, 'PlaySong')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Song service not available, waiting again...')
        self.req = PlaySong.Request()

    def send_request(self, a):
        self.req.index = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[
                ('max_deg', 120),
                ('step_deg', 20),
            ])

        self.auto_mode = False
        self.chatCount= 0
        self.mode_button_last = 0

        self.colorIdx = 0                                        # index for led on/off
        self.songIdx = 0                                         # index for buzzer song
        self.lcdIdx = 0                                          # index for oled animation

        self.control_motor0_velocity = MOTOR0_HOME
        self.control_motor1_velocity = MOTOR1_HOME
        self.control_motor2_velocity = MOTOR2_HOME
        self.control_motor3_velocity = GRIPPER_OPEN

        self.led_client = ClientAsyncLed()
        self.ani_client = ClientAsyncAni()
        self.song_client = ClientAsyncSong()

        print(' moniarm Teleop Joystick controller')
        print(msg)
        self.max_deg = self.get_parameter_or('max_deg', Parameter('max_deg', Parameter.Type.INTEGER, 120)).get_parameter_value().integer_value
        self.step_deg = self.get_parameter_or('step_deg', Parameter('step_deg', Parameter.Type.INTEGER, 20)).get_parameter_value().integer_value
        print('max ang: %s rad/s, step: %s'%
            (self.max_deg,
            self.step_deg)
        )
        print('CTRL-C to quit')

        self.robotarm = Moniarm()
        self.robotarm.home()

        self.qos = QoSProfile(depth=10)
        # generate publisher for 'cmd_vel'
        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        # generate publisher for 'ledSub
        self.timer = self.create_timer(TIMER_JOY, self.cb_timer)

        rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_control/moniarm_control/')
        self.fhandle = open(rosPath + 'automove.txt', 'w')

        self.prev_time = time()
        self.timediff = 0.0

    def cb_joy(self, joymsg):
        status = 0
        motorMsg = Int32MultiArray()
        #M0, M3 torque off by default
        motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF]

        if joymsg.buttons[0] == 1 and self.mode_button_last == 0:
            print('colorIdx: %d'%(self.colorIdx))
            self.led_client.send_request(self.colorIdx)
            self.colorIdx+=1
            if self.colorIdx >= MAX_COLOR:
                self.colorIdx=0
            self.mode_button_last = joymsg.buttons[0]

        elif joymsg.buttons[1] == 1 and self.mode_button_last == 0:
            print('songIdx: %d'%(self.songIdx))
            self.song_client.send_request(self.songIdx)
            self.songIdx+=1
            if self.songIdx >= MAX_SONG:
                self.songIdx=0
            self.mode_button_last = joymsg.buttons[1]

        elif joymsg.buttons[4] == 1 and self.mode_button_last == 0:
            print('lcdIdx: %d'%(self.lcdIdx))
            self.ani_client.send_request(self.lcdIdx)
            self.lcdIdx+=1
            if self.lcdIdx >= MAX_ANIM:
                self.lcdIdx=0
            self.mode_button_last = joymsg.buttons[4]

        elif joymsg.buttons[3] == 1 and self.mode_button_last == 0:
            status = status + 1
            if self.control_motor3_velocity == 0:
                self.control_motor3_velocity = 1
            else:
                self.control_motor3_velocity = 0
            self.mode_button_last = joymsg.buttons[3]

        # Make jostick -> /cmd_vel
        elif joymsg.axes[1] != 0:
            status = status + 1
            self.control_motor1_velocity += joymsg.axes[1] * self.max_deg / self.step_deg
        elif joymsg.axes[3] != 0:
            status = status + 1
            self.control_motor2_velocity += joymsg.axes[3] * self.max_deg / self.step_deg
        elif joymsg.axes[0] != 0:
            status = status + 1
            self.control_motor0_velocity += joymsg.axes[0] * self.max_deg / self.step_deg
        else:
            #nothing to do, then return
            return True

        motorMsg = Int32MultiArray()
        #M0, M3 torque off by default
        motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR_TOQOFF]

        #key pressed, torque
        if status == 1:
            if self.control_motor3_velocity == 0:
                motorMsg.data[3] = GRIPPER_OPEN
            else:
                motorMsg.data[3] = GRIPPER_CLOSE
            self.control_motor0_velocity = int(clamp(self.control_motor0_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))
            motorMsg.data[0] = self.control_motor0_velocity      #M0, degree

        self.control_motor1_velocity = int(clamp(self.control_motor1_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))
        self.control_motor2_velocity = int(clamp(self.control_motor2_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))
        motorMsg.data[1] = self.control_motor1_velocity          #M1, degree
        motorMsg.data[2] = self.control_motor2_velocity          #M2, degree

        self.robotarm.run(motorMsg)
        print('M0= %.2f, M1 %.2f, M2= %.2f, M3= %.2f'%(motorMsg.data[0], motorMsg.data[1],motorMsg.data[2], motorMsg.data[3]))

        timediff = time() - self.prev_time
        self.prev_time = time()
        self.fhandle.write(str(motorMsg.data[0]) + ':' + str(motorMsg.data[1]) + ':' + str(motorMsg.data[2]) + ':' + str(motorMsg.data[3])+ ':' + str(timediff) + '\n')

        status = 0

    def cb_timer(self):
        self.chatCount += 1                     # protect chattering
        if self.chatCount > MAX_CHAT:
            self.mode_button_last = 0
            self.chatCount = 0

    def __del__(self):
        print('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    teleop_joy =  TeleopJoyNode()
    rclpy.spin(teleop_joy)
    teleop_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
