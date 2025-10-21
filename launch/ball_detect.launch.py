from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, EmitEvent, RegisterEventHandler
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch
import launch_ros

def generate_launch_description():
    name_space = 'daisha'
    pkg_path = get_package_share_directory('rere_daisha_ros')

    ld = LaunchDescription()

    # realsense
    # realsense_launch_file = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'
    # )
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(realsense_launch_file),
    #     launch_arguments={
    #         'camera_namespace': name_space,
    #         'camera_name': 'D435',
    #         'enable_rgbd': 'true',
    #         'enable_sync': 'true',
    #         'align_depth.enable': 'true',
    #         'enable_color': 'true',
    #         'enable_depth': 'true',
    #         'enable_gyro': 'true',
    #         'enable_accel': 'true',
    #         'device_type': 'd435',
    #         'pointcloud.enable': 'true'   
    #     }.items()
    # )
    # ld.add_action(realsense_launch)

    # lidar

    ldlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py"]
            )
        ),
        launch_arguments={"node_ns": name_space}.items(),
    )
    # ld.add_action(ldlidar_node)

    # yolo

    # ransac
    param_file = os.path.join(pkg_path, 'config', 'ransac_params.yaml')
    ransac_node = LifecycleNode(
        package='rere_daisha_ros',
        executable='ransac_ball_node',
        namespace=name_space,
        parameters=[param_file],
        name='ransac_ball_node'
    )

    ransac_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ransac_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(ransac_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    ransac_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ransac_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(ransac_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld.add_action(ransac_node)
    ld.add_action(ransac_configure_event_handler)
    ld.add_action(ransac_activate_event_handler)

    return ld