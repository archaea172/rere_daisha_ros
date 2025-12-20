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
    x = 2.0
    y = 2.0
    z = 0.1
    theta = 0

    package_dir = get_package_share_directory("rere_daisha_ros")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    world = os.path.join(
        get_package_share_directory("rere_daisha_ros"), "worlds", "mtg_room.world"
    )

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f'-r {world}'])]
    )

    xacro_file = os.path.join(package_dir, "urdf", "swerve.xacro")
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
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/yasarobo/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/pos_1@std_msgs/msg/Float64@gz.msgs.Double',
            '/pos_2@std_msgs/msg/Float64@gz.msgs.Double',
            '/pos_3@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_node)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_robot_joint_publisher)
    ld.add_action(gz_spawn_entity)
    ld.add_action(bridge)

    return ld