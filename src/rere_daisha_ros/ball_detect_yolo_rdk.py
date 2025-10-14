import rclpy
from rclpy.node import Node

from realsense2_camera_msgs.msg import RGBD

import cv2
from cv_bridge import CvBridge
import numpy as np

class Rdk_YOLO(Node):
    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('rdk_yolo')
        self.subscriber_rs = self.create_subscription(
            RGBD,
            "/camera/camera/rgbd",
            self.rs_callback,
            10
        )
        self.subscriber_rs()
        
    def rs_callback(self, rxdata):
        # receive data
        cv_img = self.bridge.imgmsg_to_cv2(rxdata.rgb, "bgr8")
        cameramatrix_r = np.array(rxdata.rgb_camera_info.k)
        camera_matrix = cameramatrix_r.reshape((3, 3))
        distCoeffs = np.array(rxdata.rgb_camera_info.d)