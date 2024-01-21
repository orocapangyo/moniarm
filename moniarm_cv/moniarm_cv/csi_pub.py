#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CameraeNode(Node):
    def __init__(self):

        super().__init__('csicam_node')
        
        #for csi camera orientation
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        
        print("Camera Node created")
        # Create a timer that will gate the node actions twice a second
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)

    def node_callback(self):
        # Capture frame-by-frame
        ret, cv_image = self.cap.read()
        if ret == True:           
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        else:
            print("image read fail")
            self.cap.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args) 
    csiCamera = CameraeNode()
    rclpy.spin(csiCamera)
    csiCamera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
