#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace'
    ),
    DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        choices=['true', 'false'],
        description='Whether to use Gazebo simulation'
    ),
    DeclareLaunchArgument(
        'robot_type',
        default_value='omni',
        choices=['omni', 'skid', 'diff'],
        description='Robot kinematic configuration'
    ),
    DeclareLaunchArgument(
        'exploded',
        default_value='false',
        choices=['true', 'false'],
        description='Show exploded view for documentation figures'
    ),
]


def generate_launch_description():
    urdf_package = 'uvot_description'
    urdf_filename = 'robot.urdf.xacro'

    pkg_share_description = FindPackageShare(urdf_package)

    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', urdf_filename]
    )

    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf/xacro file'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start RViz'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation clock if true'
    )

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value=[namespace, '/'],
        description='Prefix for robot joints and links (por defecto = namespace + "/")'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_model, ' ',
            'prefix:=', LaunchConfiguration('prefix'), ' ',
            'robot_ns:=', namespace, ' ',
            'use_gazebo:=', LaunchConfiguration('use_gazebo'), ' ',
            'robot_type:=', LaunchConfiguration('robot_type'), ' ',
            'exploded:=', LaunchConfiguration('exploded')
        ]),
        value_type=str
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        #remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    rviz_config = os.path.join(
        get_package_share_directory('uvot_bringup'),
        'rviz',
        'uvot.rviz'
    )
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prefix_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
