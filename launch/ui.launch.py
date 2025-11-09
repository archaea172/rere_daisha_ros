from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    name_space = 'rere_daisha'

    ld = LaunchDescription()

    # converter
    tf_camera_lister = Node(
        package='rere_daisha_ros',
        executable='ball_converter_listener_node',
        namespace=name_space
    )
    ld.add_action(tf_camera_lister)

    return ld