import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import xacro
import math
import random

def generate_launch_description():
    
    x = 2.0
    y = 2.0
    theta = math.pi

    mcl_node = Node(
        package="rere_daisha_ros",
        executable="mcl_node",
        parameters=[
            {
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
                "mapDir": "/home/dev/ros2_ws/src/rere_daisha_ros/map/"
            },
        ],
        remappings=[('clock', '/world/yasarobo/clock')],
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(mcl_node)

    return ld