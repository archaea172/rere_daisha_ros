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
        package_share_directory = get_package_share_directory('rere_daisha_ros')
        weights_path = os.path.join(package_share_directory, 'weights', 'best_bayese_640x640_nv12.bin')

        self.coco_names = ['blue_ball', 'red_ball', 'yellow_ball']
        self.model = Ultralytics_YOLO_Detect_Bayese_YUV420SP(weights_path, 3, 0.4, 0.8, 16, [8, 16 ,32])

        self.subscriber_rs
        self.get_logger().info('configure finish')

        theta = np.deg2rad(-120)
        c, s = np.cos(theta), np.sin(theta)
        self.R_robot_cam = np.array([
            [1, 0,  0],
            [0, c, -s],
            [0, s,  c]
        ])
        self.t_robot_cam = np.array([[0], [-0.03], [0.1938]])
        
    def rs_callback(self, rxdata):
        # receive data
        cv_img = self.bridge.imgmsg_to_cv2(rxdata.rgb, "bgr8")
        cameramatrix_r = np.array(rxdata.rgb_camera_info.k)
        camera_matrix = cameramatrix_r.reshape((3, 3))
        distCoeffs = np.array(rxdata.rgb_camera_info.d)

        input_tensor = self.model.preprocess_yuv420sp(cv_img)
        outputs = self.model.c2numpy(self.model.forward(input_tensor))
        results = self.model.postProcess(outputs)
        # self.get_logger().info('detect finish')

        if results is not None:
            ball_size = 0.065
            objPoints = np.array([
                [-0.5,  0.5, 0],
                [ 0.5,  0.5, 0],
                [ 0.5, -0.5, 0],
                [-0.5, -0.5, 0]
            ]) * ball_size
            for class_id, score, x1, y1, x2, y2 in results:
                corner = np.array([
                    [x1, y1],
                    [x2, y1],
                    [x2, y2],
                    [x1, y2]
                ], dtype=np.float32)

                retval, rvec, tvec = cv2.solvePnP(
                    objPoints,
                    corner,
                    camera_matrix,
                    distCoeffs
                )
                p_robot = self.R_robot_cam@tvec - self.t_robot_cam
                coords = p_robot.flatten()
                x, y, z = coords[0], coords[1], coords[2]

                zahyo = str(x) + ',' + str(y) + ',' + str(z)
                self.get_logger().info('%s' %zahyo)


def main_ball_detect():
    rclpy.init()
    rdk_yolo = Rdk_YOLO()
    try:
        rclpy.spin(rdk_yolo)
    except KeyboardInterrupt:
        pass
    finally:
        rdk_yolo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()