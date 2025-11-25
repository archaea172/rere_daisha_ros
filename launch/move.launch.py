from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import math

def generate_launch_description():
    x = 0.5
    y = 0.5
    theta = 0.0

    name_space = 'rere_daisha'
    ld = LaunchDescription()

    mcl_node = Node(
        package="yasarobo2025_26",
        executable="mcl_node",
        parameters=[{
            "particleNum": 50,
            "initial_x": x,
            "initial_y": y,
            "initial_theta": theta,
            "odomNoise1": 2.0,
            "odomNoise2": 0.5,
            "odomNoise3": 2.0,
            "odomNoise4": 5.0,
            "resampleThreshold": 0.9,
            "scanStep": 5,
            "lidar_threshold": 3.0/40.0*math.pi,
            "mapDir": "/home/sunrise/rere_daisha_ws/src/yasarobo2025_26/map/"
        }],
    )
    ld.add_action(mcl_node)

    convert_node = Node(
        package='rere_daisha_ros',
        executable='convert_diff_cmd_vel',
        parameters=[{
            "wheel_distance": 0.190
        }]
    )
    ld.add_action(convert_node)

    ldlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
            )
        ),
    )
    ld.add_action(ldlidar_node)
    
    gen_path = Node(
        package="yasarobo2025_26",
        executable="gen_path",
        output="screen",
        parameters=[{
            "initial_x": x,
            "initial_y": y,
            "initial_theta": 0.0,
        }],
    )
    ld.add_action(gen_path)

    return ld
