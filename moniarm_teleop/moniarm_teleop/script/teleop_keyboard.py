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
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from .submodules.myutil import Moniarm, radiansToDegrees, trimLimits, clamp
from .submodules.myconfig import *

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

space key, s : force stop

c: Change led
u: play buzzer song
o: OLED animation

CTRL-C to quit
"""

e = """
Communications Failed
"""

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

def print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity):
    print('M0= %.2f, M1 %.2f, M2= %.2f'%(
        target_linear_velocity,
        target_linear1_velocity,
        target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:  # if variance is bigger
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    print('Param max lin: %s deg, lin step: %s deg'%
        (MAX_LIN_VEL,
        LIN_VEL_STEP_SIZE)
    )

    node = rclpy.create_node('teleop_keyboard_node')        # generate node

    robotarm = Moniarm()
    robotarm.home()

    ledpub = node.create_publisher(Int32, 'ledSub',10)      # generate publisher for 'ledSub'
    songpub = node.create_publisher(Int32, 'songSub',10)    # generate publisher for 'songpub'
    lcdpub = node.create_publisher(Int32, 'lcdSub',10)      # generate publisher for 'lcdpub'
    print('moniarm Teleop Keyboard controller')

    status = 0

    target_angular_velocity = MOTOR0_HOME
    control_angular_velocity = MOTOR0_HOME
    target_linear_velocity = MOTOR1_HOME
    control_linear_velocity = MOTOR1_HOME
    target_linear1_velocity = MOTOR2_HOME
    control_linear1_velocity = MOTOR2_HOME

    colorIdx = 0                                        # variable for saving data in ledSub's msg data field
    songIdx = 0                                         # variable for saving data in songSub's msg data field
    lcdIdx = 0
    gMsg = Int32()

    #generate variable for Twist type msg
    rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_control/moniarm_control/')
    fhandle = open(rosPath + 'automove.txt', 'w')

    prev_time = time()
    timediff = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':              # linear speed up
                target_linear_velocity = \
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)
            elif key == 'x':            # linear speed down
                target_linear_velocity = \
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)
            elif key == 'q':              # linear speed up
                target_linear1_velocity = \
                    check_linear_limit_velocity(target_linear1_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)
            elif key == 'z':            # linear speed down
                target_linear1_velocity = \
                    check_linear_limit_velocity(target_linear1_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)
            elif key == 'a':            # left angle spped up
                target_angular_velocity = \
                    check_angular_limit_velocity(target_angular_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)
            elif key == 'd':            # right angle spped up
                target_angular_velocity = \
                    check_angular_limit_velocity(target_angular_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)

            elif key == ' ' or key == 's':  # pause
                target_angular_velocity = MOTOR0_HOME
                control_angular_velocity = MOTOR0_HOME
                target_linear_velocity = MOTOR1_HOME
                control_linear_velocity =  MOTOR1_HOME
                target_linear1_velocity = MOTOR2_HOME
                control_linear1_velocity = MOTOR2_HOME
                #print_vels(target_linear_velocity, target_linear1_velocity, target_angular_velocity)

            elif key == 'c':                # led control
                print('colorIdx: %d'%(colorIdx))
                gMsg.data = colorIdx
                ledpub.publish(gMsg)
                colorIdx += 1
                if colorIdx >= MAX_COLOR:
                    colorIdx = 0

            elif key == 'u':                # play buzzer song
                print('songIdx: %d'%(songIdx))
                gMsg.data = songIdx
                songpub.publish(gMsg)
                songIdx += 1
                if songIdx >= MAX_SONG:
                    songIdx = 0

            elif key == 'o':                # play oled animation
                print('lcdIdx: %d'%(lcdIdx))
                gMsg.data = lcdIdx
                lcdpub.publish(gMsg)
                lcdIdx += 1
                if lcdIdx >= MAX_ANIM:
                    lcdIdx = 0

            else:
                #Ctrl-C, then stop working
                if (key == '\x03'):
                    break
                #no valid input, then don't control arm
                else:
                    continue

            if status == 20:
                print(msg)
                status = 0

            motorMsg = Int32MultiArray()
            motorMsg.data = [0, 0, 0, 0]
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_linear1_velocity = make_simple_profile(
                control_linear1_velocity,
                target_linear1_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = int(clamp(control_angular_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))
            control_linear_velocity = int(clamp(control_linear_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))
            control_linear1_velocity = int(clamp(control_linear1_velocity, -MAX_LIN_VEL, MAX_LIN_VEL))

            motorMsg.data[0] = control_angular_velocity         #M0, degree
            motorMsg.data[1] = control_linear_velocity          #M1, degree
            motorMsg.data[2] = control_linear1_velocity         #M2, degree
            motorMsg.data[3] = 0                                #Gripper
            robotarm.run(motorMsg)
            print('M0= %.2f, M1 %.2f, M2= %.2f'%(motorMsg.data[0], motorMsg.data[1],motorMsg.data[2]))

            timediff = time() - prev_time
            prev_time = time()
            fhandle.write(str(motorMsg.data[0]) + ':' + str(motorMsg.data[1] ) + ':' + str(motorMsg.data[2] )
                + ':' + str(motorMsg.data[3] ) + ':' + str(timediff) + '\n')

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
