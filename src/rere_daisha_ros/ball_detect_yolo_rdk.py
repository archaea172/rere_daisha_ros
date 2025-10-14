import rclpy
from rclpy.node import Node

from realsense2_camera_msgs.msg import RGBD

import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from  .ball_detect_rdk import Ultralytics_YOLO_Detect_Bayese_YUV420SP

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
        weights_path = os.path.join(package_share_directory, 'weights', 'best_bayese_640x640_nv12.bin')

        self.coco_names = ['blue_ball', 'red_ball', 'yellow_ball']
        self.model = Ultralytics_YOLO_Detect_Bayese_YUV420SP(weights_path, 3, 0.4, 0.8, 16, [8, 16 ,32])
        self.subscriber_rs()
        
    def rs_callback(self, rxdata):
        # receive data
        cv_img = self.bridge.imgmsg_to_cv2(rxdata.rgb, "bgr8")
        cameramatrix_r = np.array(rxdata.rgb_camera_info.k)
        camera_matrix = cameramatrix_r.reshape((3, 3))
        distCoeffs = np.array(rxdata.rgb_camera_info.d)

        input_tensor = self.model.preprocess_yuv420sp(cv_img)
        outputs = self.model.c2numpy(self.model.forward(input_tensor))
        results = self.model.postProcess(outputs)

        ball_size = 0.065
        objPoints = np.array([
            [-0.5,  0.5, 0],
            [ 0.5,  0.5, 0],
            [ 0.5, -0.5, 0],
            [-0.5, -0.5, 0]
        ]) * ball_size
        for class_id, score, x1, y1, x2, y2 in results:
            corner = np.array([
                [x1, y1, 0],
                [x2, y1, 0],
                [x2, y2, 0],
                [x1, y2, 0]
            ])

            retval, rvec, tvec = cv2.solvePnP(
                objPoints,
                corner,
                camera_matrix,
                distCoeffs
            )
            self.get_logger().info('%s' % self.)