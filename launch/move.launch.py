from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    name_space = 'rere_daisha'
    ld = LaunchDescription()

    