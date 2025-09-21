import cv2
import os
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO

def main_yolo_test():
    package_share_directory = get_package_share_directory('rere_daisha_ros')
    model = YOLO()