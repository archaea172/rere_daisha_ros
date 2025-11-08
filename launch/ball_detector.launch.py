from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    name_space = 'daisha'
    pkg_path = get_package_share_directory('rere_daisha_ros')

    ld = LaunchDescription()
    return ld