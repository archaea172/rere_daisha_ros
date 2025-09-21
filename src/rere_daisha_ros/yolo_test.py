import cv2
import os
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO

def main_yolo_test():
    package_share_directory = get_package_share_directory('rere_daisha_ros')
    weights_path = os.path.join(package_share_directory, 'weights', 'best.pt')
    img_path = os.path.join(package_share_directory, 'images', 'scene_inrof_2_4813_00003.png')
    model = YOLO(weights_path)

    img = cv2.imread(img_path)
    results = model(img)
    results.show()