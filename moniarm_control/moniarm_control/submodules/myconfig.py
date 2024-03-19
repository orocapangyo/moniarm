MOTOR_NOMOVE = 360
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
MOTOR1_HOME = 45
MOTOR1_PICKUP = (MOTOR1_HOME - 10)
MOTOR1_OFF = MOTOR1_HOME
MOTOR1_MIN  = (MOTOR1_HOME - 30)
MOTOR1_MAX  = (MOTOR1_HOME + 30)
MOTOR1_DIF_MIN = (MOTOR1_MIN - MOTOR1_HOME)
MOTOR1_DIF_MAX = (MOTOR1_MAX - MOTOR1_HOME)

MOTOR2_ZERO = 0
MOTOR2_HOME = 45
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

#yolo_chase or blob_chase
PICTURE_SIZE_X = 640.0
PICTURE_SIZE_Y = 480.0
CALIBPICTURE = 0.00

#object detected range, in 2%
IN_RANGE_MIN = -0.02
IN_RANGE_MAX = 0.02

#object tracking parameter
DIR_TO_STEER = -1.0
Kp = 0.2

#teleop parameter
MAX_SONG = 5
MAX_ANIM = 3
MAX_COLOR = 6
MAX_CHAT = 5
TIMER_JOY = 0.1

#teleop_keyboard parameter
MAX_LIN_VEL = 120
MAX_ANG_VEL = 120
LIN_VEL_STEP_SIZE = (MAX_LIN_VEL/20)
ANG_VEL_STEP_SIZE = (MAX_ANG_VEL/20)
