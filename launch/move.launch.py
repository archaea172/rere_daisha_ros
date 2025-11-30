from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.actions import LifecycleNode
import lifecycle_msgs.msg
import launch

import math

def generate_launch_description():
    x = 0.25
    y = 0.25
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
    # ld.add_action(mcl_node)

    convert_node = Node(
        package='rere_daisha_ros',
        executable='convert_diff_cmd_vel',
        parameters=[{
            "wheel_distance": 0.190
        }]
    )
    # ld.add_action(convert_node)

    ldlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
            )
        ),
    )
    # ld.add_action(ldlidar_node)
    
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
    # ld.add_action(gen_path)

    bridge_can = LifecycleNode(
        package='rere_daisha_ros',
        executable='ros2_can_bridge',
        name='ros2_can_bridge',
        namespace='', 
    )
    ld.add_action(bridge_can)

    
    bridge_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge_can,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(bridge_can),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    bridge_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_can,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(bridge_can),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld.add_action(bridge_configure_event_handler)
    ld.add_action(bridge_activate_event_handler)

    return ld
