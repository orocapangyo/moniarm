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
import select
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .submodules.myutil import Moniarm, clamp
from .submodules.myconfig import *
from moniarm_interfaces.srv import SetLED, PlayAni, PlaySong, Init

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
a/d : base(M0), left/light
w/x : shoulder(M1) move
q/z : Elbow(M2) move
e/c : Wrist(M3) move

l: Change led
u: play buzzer song
o: OLED animation
i: Motor intialize

CTRL-C to quit
"""

e = """
Communications Failed
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

class ClientAsyncInit(Node):
    def __init__(self):
        super().__init__('ClientAsyncInit')
        self.cli = self.create_client(Init, 'Init')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Init service not available, waiting again...')
        self.req = Init.Request()

    def send_request(self, a):
        self.req.init_mode = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:  # if valid, save to key
        key = sys.stdin.read(1)
    else:       # else, initialize
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG, MAX_ANG)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    print('Param max lin: %s deg, lin step: %s deg'%
        (MAX_ANG,
        ANG_STEP)
    )

    node = rclpy.create_node('teleop_keyboard_node')        # generate node

    robotarm = Moniarm()
    robotarm.home()
    print('moniarm Teleop Keyboard controller')

    status = 0
    control_motor0_velocity = MOTOR0_HOME
    control_motor1_velocity = MOTOR1_HOME
    control_motor2_velocity = MOTOR2_HOME
    control_motor3_velocity = MOTOR3_HOME

    colorIdx = 0                                        # index for led on/off
    songIdx = 0                                         # index for buzzer song
    lcdIdx = 0                                          # index for oled animation

    led_client = ClientAsyncLed()
    ani_client = ClientAsyncAni()
    song_client = ClientAsyncSong()
    int_client = ClientAsyncInit()

    rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_control/moniarm_control/')
    fhandle = open(rosPath + 'automove.txt', 'w')

    prev_time = time()
    timediff = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'x':              # motor1
                control_motor1_velocity = check_linear_limit_velocity(control_motor1_velocity + ANG_STEP)
                status = status + 1
            elif key == 'w':            # motor1
                control_motor1_velocity = check_linear_limit_velocity(control_motor1_velocity - ANG_STEP)
                status = status + 1
            elif key == 'z':            # motor2
                control_motor2_velocity = check_linear_limit_velocity(control_motor2_velocity + ANG_STEP)
                status = status + 1
            elif key == 'q':            # motor2
                control_motor2_velocity = check_linear_limit_velocity(control_motor2_velocity - ANG_STEP)
                status = status + 1
            elif key == 'c':            # motor3
                control_motor3_velocity = check_linear_limit_velocity(control_motor3_velocity + ANG_STEP)
                status = status + 1
            elif key == 'e':            # motor3
                control_motor3_velocity = check_linear_limit_velocity(control_motor3_velocity - ANG_STEP)
                status = status + 1
            elif key == 'a':            # motor0
                control_motor0_velocity = check_linear_limit_velocity(control_motor0_velocity + ANG_STEP)
                status = status + 1
            elif key == 'd':            # motor0
                control_motor0_velocity = check_linear_limit_velocity(control_motor0_velocity - ANG_STEP)
                status = status + 1

            elif key == 'l':            # led control
                print('colorIdx: %d'%(colorIdx))
                led_client.send_request(colorIdx)
                colorIdx += 1
                if colorIdx >= MAX_COLOR:
                    colorIdx = 0
            elif key == 'u':                # play buzzer song
                print('songIdx: %d'%(songIdx))
                song_client.send_request(songIdx)
                songIdx += 1
                if songIdx >= MAX_SONG:
                    songIdx = 0
            elif key == 'o':                # play oled animation
                print('lcdIdx: %d'%(lcdIdx))
                ani_client.send_request(lcdIdx)
                lcdIdx += 1
                if lcdIdx >= MAX_ANIM:
                    lcdIdx = 0
            elif key == 'i':                # initialize motors when motor error happens
                print('Initialize motors')
                int_client.send_request(0)

            #elif key == 'g':                # gripper
            #    status = status + 1
            #    if control_motor3_velocity == 0:
            #        control_motor3_velocity = 1
            #    else:
            #        control_motor3_velocity = 0

            else:
                #Ctrl-C, then stop working
                if (key == '\x03'):
                    break
                #no valid input, then don't control arm
                else:
                    continue

            motorMsg = Int32MultiArray()
            #M0, M3 torque off by default
            motorMsg.data = [MOTOR_TOQOFF, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME]

            #key pressed, torque
            if status == 1:
            #    if control_motor3_velocity == 0:
            #        motorMsg.data[3] = GRIPPER_OPEN
            #    else:
            #        motorMsg.data[3] = GRIPPER_CLOSE
                control_motor0_velocity = int(clamp(control_motor0_velocity, MOTOR0_MIN, MOTOR0_MAX))
                motorMsg.data[0] = control_motor0_velocity      #M0, degree

            control_motor1_velocity = int(clamp(control_motor1_velocity, MOTOR1_MIN, MOTOR1_MAX))
            control_motor2_velocity = int(clamp(control_motor2_velocity, MOTOR2_MIN, MOTOR2_MAX))
            control_motor2_velocity = int(clamp(control_motor2_velocity, MOTOR3_MIN, MOTOR3_MAX))
            motorMsg.data[1] = control_motor1_velocity
            motorMsg.data[2] = control_motor2_velocity
            motorMsg.data[3] = control_motor3_velocity

            robotarm.run(motorMsg)
            print('M0= %.2f, M1 %.2f, M2= %.2f, M3= %.2f'%(motorMsg.data[0], motorMsg.data[1],motorMsg.data[2], motorMsg.data[3]))

            timediff = time() - prev_time
            prev_time = time()
            fhandle.write(str(motorMsg.data[0]) + ':' + str(motorMsg.data[1]) + ':' + str(motorMsg.data[2]) + ':' + str(motorMsg.data[3])+ ':' + str(timediff) + '\n')

            status = 0

    except Exception as e:
        print(e)

    finally:  #
        #motor parking angle
        print('Arm parking, be careful')
        robotarm.park()

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
