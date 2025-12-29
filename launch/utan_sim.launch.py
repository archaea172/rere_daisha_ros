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
    z = 1.0
    theta = math.pi

    package_dir = get_package_share_directory("rere_daisha_ros")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    world = os.path.join(
        get_package_share_directory("rere_daisha_ros"), "worlds", "mtg_room.world"
    )
    models_path = os.path.join(package_dir, "models")  # .../share/rere_daisha_ros/models
    models_path = os.path.expanduser(models_path)
    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")

    new_gz_path = f"{gz_path}:{models_path}".strip(":")
    new_ign_path = f"{ign_path}:{models_path}".strip(":")

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f'-r {world}'])]
    )

    xacro_file = os.path.join(package_dir, "urdf", "utan_urdf.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_robot_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'robot',
                   '-allow_renaming', 'false',
                   '-x', str(x),
                   '-y', str(y),
                   '-z', str(z),
                   '-Y', str(theta)
                ],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ldlidar_node/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/yasarobo/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/arm_vertical@std_msgs/msg/Float64@gz.msgs.Double',
            '/arm_horizontal@std_msgs/msg/Float64@gz.msgs.Double',
            '/arm_yaw@std_msgs/msg/Float64@gz.msgs.Double',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_gz_path))
    ld.add_action(SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", new_ign_path))
    ld.add_action(sim_node)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_robot_joint_publisher)
    ld.add_action(gz_spawn_entity)
    ld.add_action(bridge)

    return ld