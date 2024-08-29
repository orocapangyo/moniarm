from time import sleep
import rclpy
from rclpy.node import Node
from . myconfig import *
from moniarm_interfaces.msg import CmdMotor
from rclpy.qos import qos_profile_sensor_data

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

    return int(mtr_pos)

def setArmAgles(arm, ang0, ang1, ang2, ang3, grip):
    arm.angle0 = ang0
    arm.angle1 = ang1
    arm.angle2= ang2
    arm.angle3 = ang3
    arm.grip = grip

class Moniarm(Node):
    """
    3DOF robot arm with Herkulex smart motor
    """
    def __init__(self):
        super().__init__('arm_basic_node')
        self.motorPub = self.create_publisher(CmdMotor, 'cmd_motor', qos_profile_sensor_data)
        self.motorMsg = CmdMotor()
        setArmAgles(self.motorMsg,MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN)
        self.armStatus = 'HOMING'

    def run(self, mMSG):
        self.motorMsg.angle0 = mMSG.angle0
        self.motorMsg.angle1 = mMSG.angle1
        self.motorMsg.angle2 = mMSG.angle2
        self.motorMsg.angle3 = mMSG.angle3
        self.motorMsg.grip =  mMSG.grip
        self.motorPub.publish(self.motorMsg)

    def park(self):
        print("Parking...")
        self.motorMsg.grip = GRIPPER_OPEN
        self.motorMsg.angle0 = MOTOR_TOQON
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.1)

        self.motorMsg.angle1 = (MOTOR1_OFF + 20)
        self.motorMsg.angle2 = MOTOR2_OFF
        self.motorMsg.angle3 = MOTOR3_OFF
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        self.motorMsg.angle1 = MOTOR1_OFF
        self.motorPub.publish(self.motorMsg)
        sleep(0.7)
        self.motorMsg.angle0 = MOTOR0_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(2.0)

        self.motorMsg.angle0 = MOTOR_TOQOFF
        self.motorMsg.angle1 = MOTOR_TOQOFF
        self.motorMsg.angle2 = MOTOR_TOQOFF
        self.motorMsg.angle3 = MOTOR_TOQOFF
        self.motorPub.publish(self.motorMsg)
        sleep(0.2)
        print("Parking Done")

    def home(self):
        print("Homing...")
        #torque on at first except MOTOR0
        self.motorMsg.grip = GRIPPER_OPEN
        self.motorMsg.angle0 = MOTOR_TOQON
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.1)

        self.motorMsg.angle2 = 90
        self.motorPub.publish(self.motorMsg)
        sleep(0.7)

        self.motorMsg.angle1 = MOTOR1_HOME
        self.motorMsg.angle3 = MOTOR3_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        self.motorMsg.angle2 = MOTOR2_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        self.motorMsg.angle0 = MOTOR0_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(1.5)
        print("Homing Done")

    def zero(self):
        print("Zeroing...")
        #torque on at first except MOTOR0
        self.motorMsg.grip = GRIPPER_OPEN
        self.motorMsg.angle0 = MOTOR_TOQOFF
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.1)
        self.motorMsg.angle1 = MOTOR1_HOME - 10
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle2 = MOTOR2_HOME - 20
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle3 = MOTOR3_HOME - 20
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle2 = MOTOR2_ZERO
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle1 = MOTOR1_ZERO
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle3 = MOTOR3_ZERO
        self.motorPub.publish(self.motorMsg)
        sleep(0.8)
        self.motorMsg.angle0 = MOTOR_TOQON
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(0.2)
        print("Zoering Done")

    def picknplace(self, object, down):
        if down == 1:
            #move to pick up postion
            self.motorMsg.angle0 = MOTOR_TOQON
            self.motorMsg.angle1 = MOTOR_TOQON
            self.motorMsg.angle2 = MOTOR_TOQON
            self.motorMsg.angle3 = MOTOR3_PICKUP
            self.motorPub.publish(self.motorMsg)
            sleep(1.0)
            self.motorMsg.angle1 = MOTOR1_PICKUP
            self.motorPub.publish(self.motorMsg)
            sleep(1.0)

        #grap action
        self.motorMsg.grip = GRIPPER_CLOSE
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #lift up
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR2_HOME
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.angle1 = (MOTOR1_HOME - 20)
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #move to place position
        if object == 1:
            self.motorMsg.angle0 = MOTOR0_PLACE1
        else:
            self.motorMsg.angle0 = MOTOR0_PLACE2
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #move down postion
        self.motorMsg.angle1 = MOTOR1_PICKUP
        self.motorMsg.angle3 = MOTOR3_PICKUP
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.grip = GRIPPER_OPEN
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        #place action

        #lift up
        self.motorMsg.angle2 = MOTOR2_HOME
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)
        self.motorMsg.angle1 = MOTOR1_HOME
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

        #Home
        self.motorMsg.angle0 = MOTOR0_HOME
        self.motorMsg.angle1 = MOTOR_TOQON
        self.motorMsg.angle2 = MOTOR_TOQON
        self.motorMsg.angle3 = MOTOR_TOQON
        self.motorPub.publish(self.motorMsg)
        sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Moniarm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

