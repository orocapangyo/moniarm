from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

#turn left or right of base
MOTOR0_ZERO = 0
MOTOR0_HOME = 0
MOTOR0_OFF = MOTOR0_HOME
YAW_MIN = -150
YAW_MAX = 150
MOTOR0_PLACE1 = YAW_MIN
MOTOR0_PLACE2 = YAW_MAX

#forward or backward
#roll, 2 DOF
ROLL_TOTAL_MAX = 90
ROLL_TOTAL_MIN = -90

MOTOR1_ZERO = 0
MOTOR1_HOME = 90
MOTOR1_PICKUP = (MOTOR1_HOME - 10)
MOTOR1_OFF = MOTOR1_HOME
MOTOR1_MIN  = (MOTOR1_HOME - 30)
MOTOR1_MAX  = (MOTOR1_HOME + 30)
MOTOR1_DIF_MIN = (MOTOR1_MIN - MOTOR1_HOME)
MOTOR1_DIF_MAX = (MOTOR1_MAX - MOTOR1_HOME)

MOTOR2_ZERO = 0
MOTOR2_HOME = 0
MOTOR2_PICKUP = (MOTOR2_HOME - 30)
MOTOR2_OFF = (MOTOR2_HOME + 20)

MOTOR2_MIN  = (MOTOR2_HOME - 30)
MOTOR2_MAX  = (MOTOR2_HOME + 30)
MOTOR2_DIF_MIN = (MOTOR2_MIN - MOTOR2_HOME)
MOTOR2_DIF_MAX = (MOTOR2_MAX - MOTOR2_HOME)

#gripper open/close
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1
GRIPPER_ZERO = GRIPPER_OPEN
GRIPPER_HOME = GRIPPER_OPEN
GRIPPER_OFF = GRIPPER_OPEN
GRIPPER_MIN = GRIPPER_OPEN
GRIPPER_MAX = GRIPPER_CLOSE

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
        self.motorPub = self.create_publisher(Int32MultiArray, 'motorSub',10)
        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [0, 0, 0, 0]
        self.armStatus = "Homing"

    def run(self, mMSG):
        self.motorMsg.data[0] = mMSG.data[0]
        self.motorMsg.data[1] = mMSG.data[1]
        self.motorMsg.data[2] = mMSG.data[2]
        self.motorMsg.data[3] = mMSG.data[3]
        self.motorPub.publish(self.motorMsg)

    def park(self):
        self.motorMsg.data[0] = MOTOR0_OFF
        self.motorMsg.data[1] = MOTOR1_OFF
        self.motorMsg.data[2] = MOTOR2_OFF
        self.motorMsg.data[3] = GRIPPER_OFF
        self.motorPub.publish(self.motorMsg)

    def home(self):
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = GRIPPER_HOME
        self.motorPub.publish(self.motorMsg)

    def picknplace(self, object):
        #pose for pikcing up
        self.motorMsg.data[1] = MOTOR1_PICKUP
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #grab object
        self.motorMsg.data[3] = GRIPPER_CLOSE
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #lift up a little
        self.motorMsg.data[2] = MOTOR2_PICKUP
        self.motorPub.publish(self.motorMsg)

        #move to place position
        if object == 1:
            self.motorMsg.data[0] = MOTOR0_PLACE1
        else:
            self.motorMsg.data[0] = MOTOR0_PLACE2
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #move down for placing
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #place object
        self.motorMsg.data[3] = GRIPPER_OPEN
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #lift up
        self.motorMsg.data[1] = MOTOR1_PICKUP
        self.motorPub.publish(self.motorMsg)
        sleep(0.5)

        #move to home postion
        self.motorMsg.data[0] = MOTOR0_HOME
        self.motorMsg.data[1] = MOTOR1_HOME
        self.motorMsg.data[2] = MOTOR2_HOME
        self.motorMsg.data[3] = GRIPPER_OPEN
        sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Moniarm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

