from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    name_space = 'daisha'

    ld = LaunchDescription()

    # realsense
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_namespace': name_space,
            'camera_name': 'D435',
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'device_type': 'd435',
            'pointcloud.enable': 'true'   
        }.items()
    )
    ld.add_action(realsense_launch)

    # lidar
    ldlidar_node = Node(
        package='ldlidar_node',
        executable='ldlidar_node',
        namespace='daisha',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'topic_name': 'scan',
            'lidar_frame': 'ldlidar_link',
            'range_threshold': 0.005
        }]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace='daisha',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['ldlidar_node']}]
    )
    ld.add_action(ldlidar_node)
    ld.add_action(lifecycle_manager)

    # yolo

    # ransac

    return ld