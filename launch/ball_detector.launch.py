from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    name_space = 'rere_daisha'
    pkg_path = get_package_share_directory('rere_daisha_ros')

    ld = LaunchDescription()

    # yolo
    yolo_rdk_node = Node(
        package='rere_daisha_ros',
        executable='ball_detect_yolo_rdk.py',
        namespace=name_space
    )
    ld.add_action(yolo_rdk_node)

    # カメラ用のtf
    tf2_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_publisher',
            arguments=[
                '0.2', '0.0', '0.1',
                '0.0', '-0.785398', '0.0',
                'base_link', 
                'camera_link'
            ]
    )
    ld.add_action(tf2_camera_node)

    return ld