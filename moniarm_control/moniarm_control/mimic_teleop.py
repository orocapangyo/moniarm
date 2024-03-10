#!/usr/bin/python

from time import sleep, time
import sys
import rclpy
from .submodules.myutil import clamp, Moniarm, radiansToDegrees, trimLimits
from std_msgs.msg import Int32, Int32MultiArray
from rclpy.node import Node
import os

msg = """
Mimic Human's operation!
Caution: need to run teleop first
"""

def main():
    rclpy.init()

    node = rclpy.create_node('mimic_teleop_node')        # generate node

    robotarm = Moniarm()
    robotarm.home()

    print('moniarm Teleop Keyboard controller')

    try:
        print(msg)
        rosPath = os.path.expanduser('~/ros2_ws/src/moniarm/moniarm_control/moniarm_control/')
        moveHistory = open(rosPath + 'automove.txt', 'r')

        motorMsg = Int32MultiArray()
        motorMsg.data = [0, 0, 0, 0]

        while(1):
            # Get next line from file
            line = moveHistory.readline()
            # if line is empty, end of file is reached
            if not line:
                break

            motor0, motor1, motor2, motor3, motor4, motor5, time_diff = line.split(':')
            motorMsg.data[0] = int(motor0)
            motorMsg.data[1] = int(motor2)
            motorMsg.data[2] = int(motor3)
            motorMsg.data[3] = int(motor3)

            try:
                sleep(float(time_diff))
            except KeyboardInterrupt:
                break

            robotarm.run(motorMsg)
            sys.stdout.write(str(motor0)+':'+str(motor1)+':'+str(motor2)+ ':' + str(time_diff))
            sys.stdout.flush()

    except Exception as e:
        print(e)

    finally:  #
        #motor parking angle
        robotarm.park()

if __name__ == '__main__':
    main()
