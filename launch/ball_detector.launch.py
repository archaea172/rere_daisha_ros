from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import numpy as np

def generate_launch_description():
    name_space = 'rere_daisha'
    pkg_path = get_package_share_directory('rere_daisha_ros')

    ld = LaunchDescription()

    # yolo
    yolo_rdk_node = Node(
        package='rere_daisha_ros',
        executable='ball_detect_yolo.py',
        namespace=name_space
    )
    # ld.add_action(yolo_rdk_node)

    # カメラ用のtf
    rad = -120
    theta = np.deg2rad(rad)
    tf2_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_publisher',
            arguments=[
                '0.0', '-0.03', '0.1938',
                '0.0', '0.0', str(theta),
                'base_link', 
                'camera_link'
            ]
    )
    ld.add_action(tf2_camera_node)

    return ld