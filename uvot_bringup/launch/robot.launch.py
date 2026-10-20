from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    robot_type = LaunchConfiguration('robot_type')
    serial_port = LaunchConfiguration('serial_port')

    bringup_share = get_package_share_directory('uvot_bringup')
    lidar_share = get_package_share_directory('uvot_lidar')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'robot_type': robot_type,
            'use_gazebo': 'false',
        }.items()
    )

    odom_to_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'odom_to_tf.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lidar_share,
                'launch',
                'lidar_with_filter.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'serial_port': serial_port,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot1',
            description='Namespace of the UVOT robot instance'
        ),

        DeclareLaunchArgument(
            'robot_type',
            default_value='omni',
            choices=['omni', 'skid', 'diff'],
            description='UVOT locomotion configuration'
        ),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/lidar',
            description='Serial port of the RPLIDAR'
        ),

        robot_description_launch,
        odom_to_tf_launch,
        lidar_launch,
    ])
