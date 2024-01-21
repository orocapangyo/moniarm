#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from geometry_msgs.msg import Twist
from .submodules.myutil import clamp, PCA9685, PWMThrottle, PWMThrottle2Wheel, PWMThrottleHat, PWMSteering

global speed_pulse
global steering_pulse

class VehicleNode(Node):
    def __init__(self):

        super().__init__('motor_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('has_steer', 1),
                ('isDCSteer', 0),
                ('steer_center', 380),
                ('steer_limit', 100),
                ('speed_center', 0),
                ('speed_limit', 4096),
                ('i2cSteer', 64),
                ('i2cThrottle', 96),
           ])
        self.get_logger().info("Setting Up the Node...")

        self.hasSteer = self.get_parameter_or('has_steer').get_parameter_value().integer_value
        self.isDCSteer = self.get_parameter_or('isDCSteer').get_parameter_value().integer_value  
        self.STEER_DIR = self.get_parameter_or('steer_dir').get_parameter_value().integer_value

        self.STEER_CENTER = self.get_parameter_or('steer_center').get_parameter_value().integer_value 
        self.STEER_LIMIT = self.get_parameter_or('steer_limit').get_parameter_value().integer_value
        self.SPEED_CENTER = self.get_parameter_or('speed_center').get_parameter_value().integer_value 
        self.SPEED_LIMIT = self.get_parameter_or('speed_limit').get_parameter_value().integer_value

        self.i2cSteer = self.get_parameter_or('i2cSteer').get_parameter_value().integer_value
        self.i2cThrottle = self.get_parameter_or('i2cThrottle').get_parameter_value().integer_value

        print('hasSteer: %s, i2cSteer: %s, i2cThrottle: %s'%
            (self.hasSteer,
            self.i2cSteer,
            self.i2cThrottle)
        )

        #RCcar which has steering
        if self.hasSteer == 1:
            #Steer with DC motor driver 
            if self.isDCSteer == 1:
                steer_controller = PCA9685(channel=0, address=self.i2cSteer, busnum=1)
                self._steering = PWMSteering(controller=steer_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            #Steer with servo motor
            else:
                self._steering = PCA9685(channel=0, address=self.i2cSteer, busnum=1)
            self.get_logger().info("Steering Controller Awaked!!") 
            
            throttle_controller = PCA9685(channel=0, address=self.i2cThrottle, busnum=1)
            if self.isDCSteer == 1:
                #Throttle with Motorhat
                self._throttle = PWMThrottleHat(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)             
            else:
                #Throttle with Jetracer
                self._throttle = PWMThrottle(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            self.get_logger().info("Throttle Controller Awaked!!") 
            
        #2wheel RCcar
        else:
            throttle_controller = PCA9685(channel=0, address=self.i2cSteer, busnum=1)
            self._throttle = PWMThrottle2Wheel(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
            self.get_logger().info("2wheel Throttle Controller Awaked!!")         

        self._teleop_sub = self.create_subscription(Twist, 'cmd_vel', self.cb_teleop, 10)       
        self.get_logger().info("Keyboard Subscriber Awaked!! Waiting for keyboard/joystick...")

    def cb_teleop(self, msg):

        #self.get_logger().info("Received a /cmd_vel message!")
        self.get_logger().info("Components: [%0.2f, %0.2f]"%(msg.linear.x, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        # linear.x: 0, 0+step...1.0, step=0.1
        speed_pulse = self.SPEED_CENTER + msg.linear.x*self.SPEED_LIMIT 
        speed_pulse = clamp(speed_pulse, -self.SPEED_LIMIT+1, self.SPEED_LIMIT-1)

        # angular.z: 0, 0+step...1.0, step=0.1
        steering_pulse = self.STEER_CENTER - msg.angular.z*self.STEER_LIMIT
        steering_pulse = clamp(steering_pulse, self.STEER_CENTER - self.STEER_LIMIT, self.STEER_CENTER + self.STEER_LIMIT)

        if self.hasSteer == 1:
            print(
                "speed_pulse : "
                + str(speed_pulse)
                + " / "
                + "steering_pulse : "
                + str(steering_pulse)
            )
        else:
             print(
                "speed_pulse : "
                + str(speed_pulse)
                + " / "
                + "steer % : "
                + str(steering_pulse*100/self.STEER_LIMIT)
            )

        if self.hasSteer == 1:
            self._throttle.run(speed_pulse)
            self._steering.run(steering_pulse)
        else:
            self._throttle.run(speed_pulse,steering_pulse)

def main(args=None):
    rclpy.init(args=args) 
    myCar = VehicleNode()
    rclpy.spin(myCar)
    myCar.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()