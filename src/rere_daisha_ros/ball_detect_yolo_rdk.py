import os
from typing import Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD

from rere_daisha_msgs.msg import BallPosition, BallPositionArray

from .ball_detect_rdk import Ultralytics_YOLO_Detect_Bayese_YUV420SP


class Rdk_YOLO(Node):
    """ROS2 node that publishes YOLO-based ball positions."""

    NODE_NAME = "rdk_yolo"
    CAMERA_TOPIC = "/camera/camera/rgbd"
    OUTPUT_TOPIC = "ball_position_yolo"
    FRAME_ID = "camera_frame"
    BALL_DIAMETER_M = 0.065
    MODEL_CLASSES = ["blue_ball", "red_ball", "yellow_ball"]
    MODEL_THRESHOLD = 0.4
    MODEL_IOU = 0.8
    MODEL_BATCH = 16
    MODEL_STRIDES = [8, 16, 32]

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.bridge = CvBridge()
        self.subscriber_rs = self.create_subscription(
            RGBD,
            self.CAMERA_TOPIC,
            self.rs_callback,
            10
        )
        self.publisher_ball_pos = self.create_publisher(
            BallPositionArray,
            self.OUTPUT_TOPIC,
            10
        )
        self.model = self._create_model()
        self.obj_points = self._build_obj_points()

        self.get_logger().info("RDK YOLO configured")

    def _create_model(self) -> Ultralytics_YOLO_Detect_Bayese_YUV420SP:
        package_share_directory = get_package_share_directory("rere_daisha_ros")
        weights_path = os.path.join(
            package_share_directory,
            "weights",
            "best_bayese_640x640_nv12.bin",
        )
        return Ultralytics_YOLO_Detect_Bayese_YUV420SP(
            weights_path,
            len(self.MODEL_CLASSES),
            self.MODEL_THRESHOLD,
            self.MODEL_IOU,
            self.MODEL_BATCH,
            self.MODEL_STRIDES,
        )

    def _build_obj_points(self) -> np.ndarray:
        """Return square corner points scaled to the ball diameter."""
        square = np.array(
            [
                [-0.5, 0.5, 0.0],
                [0.5, 0.5, 0.0],
                [0.5, -0.5, 0.0],
                [-0.5, -0.5, 0.0],
            ],
            dtype=np.float32,
        )
        return square * self.BALL_DIAMETER_M

    def rs_callback(self, rxdata: RGBD) -> None:
        cv_img = self._rgbd_to_cv(rxdata)
        camera_matrix, dist_coeffs = self._camera_parameters(rxdata)
        detections = self._run_model(cv_img) or []
        msg = self._build_ball_positions(detections, camera_matrix, dist_coeffs)
        self.publisher_ball_pos.publish(msg)

    def _rgbd_to_cv(self, rxdata: RGBD) -> np.ndarray:
        return self.bridge.imgmsg_to_cv2(rxdata.rgb, "bgr8")

    def _camera_parameters(self, rxdata: RGBD) -> Tuple[np.ndarray, np.ndarray]:
        camera_matrix = np.array(rxdata.rgb_camera_info.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(rxdata.rgb_camera_info.d, dtype=np.float64)
        return camera_matrix, dist_coeffs

    def _run_model(self, cv_img: np.ndarray) -> Optional[Sequence[Sequence[float]]]:
        input_tensor = self.model.preprocess_yuv420sp(cv_img)
        outputs = self.model.c2numpy(self.model.forward(input_tensor))
        return self.model.postProcess(outputs)

    def _build_ball_positions(
        self,
        detections: Iterable[Sequence[float]],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> BallPositionArray:
        msg = BallPositionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.FRAME_ID

        for detection in detections:
            ball = self._ball_from_detection(
                detection, camera_matrix, dist_coeffs
            )
            if ball:
                msg.balls.append(ball)
        return msg

    def _ball_from_detection(
        self,
        detection: Sequence[float],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> Optional[BallPosition]:
        class_id, _score, x1, y1, x2, y2 = detection
        corners = np.array(
            [
                [x1, y1],
                [x2, y1],
                [x2, y2],
                [x1, y2],
            ],
            dtype=np.float32,
        )
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            self.get_logger().warning("solvePnP failed for detection, skipping.")
            return None

        pose = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
        ball_position = BallPosition()
        ball_position.position = pose
        ball_position.class_id = int(class_id)
        return ball_position


def main_ball_detect() -> None:
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
