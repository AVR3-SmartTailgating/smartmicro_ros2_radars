import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch bunker_mini base, robot description, radar, and filter nodes."""

    # --- Static TF: base_link -> umrr (radar mount position on bunker) ---
    # Adjust x, y, z, yaw, pitch, roll to match physical radar mounting
    radar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_umrr',
        arguments=['0.3', '0', '0.2', '0', '0', '0', 'base_link', 'umrr'],
        output='screen'
    )

    # --- Bunker base launch (motor control + odometry) ---
    # bunker_base_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('bunker_base'),
    #             'launch', 'bunker_base.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'is_bunker_mini': 'true',
    #         'port_name': 'can0',
    #     }.items()
    # )

    # --- Bunker description launch (URDF + robot_state_publisher) ---
    bunker_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bunker_description'),
                'launch', 'bunker_description.launch.py'
            )
        )
    )

    # --- Radar launch (radar driver + filter) ---
    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('umrr_ros2_driver'),
                'launch', 'radar.launch.py'
            )
        )
    )

    # --- Static TF: base_link -> d435 (camera 10cm below radar) ---
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d435',
        arguments=['0.3', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    # --- RealSense D435 launch ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        )
    )

    # --- Cluster visualization node ---
    cluster_viz_params = os.path.join(
        get_package_share_directory('umrr_ros2_driver'),
        'param', 'radar_cluster_viz.params.yaml')

    cluster_viz_node = Node(
        package='umrr_ros2_driver',
        executable='radar_cluster_viz_node_exe',
        name='radar_cluster_viz',
        parameters=[cluster_viz_params],
        output='screen'
    )

    return LaunchDescription([
        # bunker_base_launch,
        radar_tf,
        camera_tf,
        bunker_description_launch,
        radar_launch,
        realsense_launch,
        cluster_viz_node,
    ])
