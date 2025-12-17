import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import xacro
import math
import random

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    world = os.path.join(
        get_package_share_directory("rere_daisha_ros"), "worlds", "mtg_room.world"
    )

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r {world}'])]
    )

    ld = LaunchDescription()
    ld.add_action(sim_node)

    return ld