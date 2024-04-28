from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from . myconfig import *

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

# Convert radians to degreees
def radiansToDegrees(position_radians):
    position_radians = position_radians * 57.2958
    return int(position_radians)

# Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
def trimLimits(mtr_pos):
    if(mtr_pos > 180):
      mtr_pos = 180

    if(mtr_pos < -180):
      mtr_pos = -180

    return mtr_pos

class Moniarm(Node):
    """
    3DOF robot arm with Herkulex smart motor
    """
    def __init__(self):
        super().__init__('arm_basic_node')
        self.motorPub = self.create_publisher(Int32MultiArray, 'cmd_motor',10)
        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME]
        self.armStatus = "Homing"

    def run(self, mMSG):
        self.motorMsg.data[0] = mMSG.data[0]
        self.motorMsg.data[1] = mMSG.data[1]
        self.motorMsg.data[2] = mMSG.data[2]
        self.motorMsg.data[3] = mMSG.data[3]
        self.motorPub.publish(self.motorMsg)

    def park(self):
        print("Parking...")
        self.motorMsg.data[0] = MOTOR0_OFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = (MOTOR1_OFF + 20)
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.5)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_OFF
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.9)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR3_OFF
        self.motorPub.publish(self.motorMsg)
        sleep(0.4)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_OFF
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.7)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQOFF
        self.motorMsg.data[2] = MOTOR_TOQOFF
        self.motorMsg.data[3] = MOTOR_TOQOFF
        self.motorPub.publish(self.motorMsg)
        sleep(0.2)
        print("Parking Done")
    def home(self):
        print("Homing...")
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR3_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = (MOTOR1_HOME - 20)
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.5)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.6)
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQOFF
        self.motorPub.publish(self.motorMsg)
        sleep(0.2)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.2)
        print("Homing Done")
    def picknplace(self, object):
        #move to pick up postion
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR3_PICKUP
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_PICKUP
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_PICKUP
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        #grap action

        #lift up
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #move to place position
        if object == 1:
            self.motorMsg.data[0] = MOTOR0_PLACE1
        else:
            self.motorMsg.data[0] = MOTOR0_PLACE2
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON 
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #move to pick up postion
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR3_PICKUP
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_PICKUP
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_PICKUP
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        #place action

        #lift up
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.data[0] = MOTOR_TOQOFF
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #Home
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR_TOQON
        self.motorMsg.data[2] = MOTOR_TOQON
        self.motorMsg.data[3] = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Moniarm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

