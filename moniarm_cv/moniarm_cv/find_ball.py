#!/usr/bin/env python3

"""
ON THE RASPI: roslaunch raspicam_node camerav2_320x240.launch enable_raw:=true

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
PUBLISHES TO:
    /blob/image_blob : image with detected blob and search window
    /blob/image_mask : masking
    /blob/point_blob : blob position in adimensional values wrt. camera frame

"""
import sys
import rclpy
from rclpy.node import Node
import cv2
import time
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import PointStamped
from cv_bridge              import CvBridge, CvBridgeError
from .submodules.blob_detector import *

class BlobDetector(Node):

    def __init__(self, blur=15, blob_params=None, detection_window=None) :
        super().__init__('blob_detect_node')
        self.declare_parameters(
        namespace='',
        parameters=[
            ('blob_min', [39, 81, 71]),
            ('blob_max',[75, 255, 255]),
        ])

        self.thr_min = self.get_parameter_or("blob_min").get_parameter_value().integer_array_value
        self.thr_max = self.get_parameter_or("blob_max").get_parameter_value().integer_array_value

        print(*self.thr_min, sep = ',')
        print(*self.thr_max, sep = ',')

        self.set_threshold(tuple(self.thr_min), tuple(self.thr_max))
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        self._t0 = time.time()

        print (">> Publishing image to topic image_blob")
        self.image_pub = self.create_publisher(Image, "/blob/image_blob", 10)
        self.mask_pub = self.create_publisher(Image, "/blob/image_mask", 10)
        self.blob_pub = self.create_publisher(PointStamped, "/blob/point_blob", 10)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image_raw",self.callback, qos_profile_sensor_data)
        print ("<< Subscribed to topic /image_raw")
        self.blob_point = PointStamped()

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        self._blur = blur

    def set_blob_params(self, blob_params):
        self._blob_params = blob_params

    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect blobs
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            blob_params=self._blob_params, search_window=self.detection_window )
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)
            cv_image    = draw_keypoints(cv_image, keypoints)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)

            for i, keyPoint in enumerate(keypoints):
                #--- Here you can implement some tracking algorithm to filter multiple detections
                #--- We are simply getting the first result
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))

                #--- Find x and y position in camera adimensional frame
                x, y = get_blob_relative_position(cv_image, keyPoint)

                self.blob_point.point.x = x
                self.blob_point.point.y = y
                self.blob_point.header.stamp = self.get_clock().now().to_msg()
                self.blob_pub.publish(self.blob_point)
                break

            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()

def main(args=None):
    blur     = 5
    min_size = 10
    max_size = 40

    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.1
    x_max   = 0.9
    y_min   = 0.1
    y_max   = 0.9

    detection_window = [x_min, y_min, x_max, y_max]

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 200

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 10000
    params.maxArea = 100000      # 640 * 480 = 307,200

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.4

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.2

    rclpy.init(args=args)

    blob = BlobDetector(blur, params, detection_window)

    rclpy.spin(blob)
    blob.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
