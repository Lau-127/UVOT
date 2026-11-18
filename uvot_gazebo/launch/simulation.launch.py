import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    gazebo_pkg = get_package_share_directory('uvot_gazebo')
    description_pkg = get_package_share_directory('uvot_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    description_share_parent = os.path.dirname(description_pkg)

    # ============================================================
    # Launch arguments
    # ============================================================

    namespace = LaunchConfiguration('namespace')
    robot_type = LaunchConfiguration('robot_type')

    # ============================================================
    # Gazebo resource path
    # ============================================================

    current_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    resource_paths = [
        path
        for path in [
            current_resource_path,
            description_share_parent,
        ]
        if path
    ]

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths)
    )

    # ============================================================
    # Gazebo system plugin path
    # ============================================================

    uvot_gazebo_prefix = get_package_prefix('uvot_gazebo')
    uvot_plugin_path = os.path.join(
        uvot_gazebo_prefix,
        'lib'
    )

    current_plugin_path = os.environ.get(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ''
    )

    plugin_paths = [
        path
        for path in [
            uvot_plugin_path,
            current_plugin_path,
        ]
        if path
    ]

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.pathsep.join(plugin_paths)
    )

    # ============================================================
    # Files
    # ============================================================

    world = os.path.join(
        gazebo_pkg,
        'worlds',
        'empty.sdf'
    )

    xacro_file = os.path.join(
        gazebo_pkg,
        'urdf',
        'robot_gazebo.urdf.xacro'
    )

    # ============================================================
    # Robot description
    # ============================================================

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' robot_ns:=', namespace,
            ' robot_type:=', robot_type,
            ' exploded:=false'
        ]),
        value_type=str
    )

    # ============================================================
    # Gazebo
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world}'
        }.items()
    )

    # ============================================================
    # ROS <-> Gazebo bridge
    #
    # /clock, IMU (/<ns>/imu) y ground truth (/model/<ns>/odometry_gt).
    # cmd_vel remains entirely on the ROS 2 side.
    # ============================================================

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ['/', namespace, '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            ['/model/', namespace,
             '/odometry_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ============================================================
    # Robot State Publisher
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ============================================================
    # Spawn robot
    # ============================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', namespace,
            '-topic', [namespace, '/robot_description'],
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.001',
        ]
    )

    # ============================================================
    # ros2_control spawners
    # ============================================================

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'joint_state_broadcaster',
            '-c', 'controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen'
    )

    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'mecanum_controller',
            '-c', 'controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", robot_type, "' == 'omni'"
            ])
        )
    )

    skid_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'skid_controller',
            '-c', 'controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", robot_type, "' == 'skid'"
            ])
        )
    )

    diff_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'diff_controller',
            '-c', 'controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", robot_type, "' == 'diff'"
            ])
        )
    )

    # Spawn JSB only after Gazebo has successfully created the robot
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )

    start_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                mecanum_controller_spawner,
                skid_controller_spawner,
                diff_controller_spawner,
            ]
        )
    )

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='robot1',
            description='Robot namespace'
        ),

        DeclareLaunchArgument(
            'robot_type',
            default_value='omni',
            choices=['omni', 'skid', 'diff'],
            description='UVOT locomotion configuration'
        ),

        # Environment must be configured before Gazebo starts
        gz_resource_path,
        gz_plugin_path,

        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,

        start_joint_state_broadcaster,
        start_drive_controller,
    ])