from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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

    ldlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
            )
        ),
        launch_arguments={"node_ns": name_space}.items(),
    )
    ld.add_action(ldlidar_node)

    # yolo

    # ransac

    return ld