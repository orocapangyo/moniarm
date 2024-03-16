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
MAX_LIN_VEL = 1.57
MAX_ANG_VEL = 1.57
LIN_VEL_STEP_SIZE = (MAX_LIN_VEL/20)
ANG_VEL_STEP_SIZE = (MAX_ANG_VEL/20)
