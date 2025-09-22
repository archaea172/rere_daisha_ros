import cv2
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from ultralytics import YOLO

def main_yolo_test():
    package_share_directory = get_package_share_directory('rere_daisha_ros')
    weights_path = os.path.join(package_share_directory, 'weights', 'best.pt')
    img_path = os.path.join(package_share_directory, 'images', 'scene_inrof_2_4813_00003.png')
    model = YOLO(weights_path)

    img = cv2.imread(img_path)
    results = model(img)
    
    annotated_img = results[0].plot()
    cv2.resize(annotated_img, (100, 100))
    cv2.imshow("yolo", annotated_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    ballsize = 10
    obj_points = np.array([
        [-0.5,  0.5, 0],
        [ 0.5,  0.5, 0],
        [ 0.5, -0.5, 0],
        [-0.5, -0.5, 0]
    ])*ballsize

    height = results[0].orig_shape[0]
    width = results[0].orig_shape[1]
    cx = width/2
    cy = height/2
    fx = width
    fy = width
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    for box in results[0].boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        class_id = int(box.cls[0].cpu().numpy())
        label = model.names[class_id]
        score = float(box.conf[0].cpu().numpy())
        if score < 0.25:
            continue

        x1, y1, x2, y2 = xyxy
        image_points = np.array([
            [x1, y1],
            [x2, y1], 
            [x2, y2],
            [x1, y2]
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points, 
            image_points, 
            camera_matrix, 
            dist_coeffs
        )
        
        print(x1, y1, x2, y2)