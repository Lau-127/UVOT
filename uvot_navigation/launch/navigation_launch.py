# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _build_nodes(context, *args, **kwargs):

    ns = LaunchConfiguration('namespace').perform(context)
    raw_params_path = LaunchConfiguration('params_file').perform(context)

    robot_type = LaunchConfiguration('robot_type').perform(context)
    map_file = LaunchConfiguration('map').perform(context)

    if robot_type == 'omni':
        amcl_motion_model = 'nav2_amcl::OmniMotionModel'
        mppi_motion_model = 'Omni'

        max_velocity = '[0.2, 0.2, 0.1]'
        min_velocity = '[-0.2, -0.2, -0.1]'
        max_accel = '[0.1, 0.1, 0.2]'
        max_decel = '[-0.1, -0.1, -0.2]'

    elif robot_type in ('diff', 'skid'):
        amcl_motion_model = 'nav2_amcl::DifferentialMotionModel'
        mppi_motion_model = 'DiffDrive'

        max_velocity = '[0.5, 0.0, 2.0]'
        min_velocity = '[-0.5, 0.0, -2.0]'
        max_accel = '[1.5, 0.0, 2.2]'
        max_decel = '[-1.5, 0.0, -2.2]'

    else:
        raise RuntimeError(
            f"Unsupported robot_type '{robot_type}'. "
            "Expected: omni, diff or skid."
        )

    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (ns, '/', container_name.perform(context))

    with open(raw_params_path, 'r') as f:
        yaml_text = f.read()
    yaml_text = yaml_text.replace('__ROBOT_NS__', ns)
    yaml_text = yaml_text.replace(
        '__MAP_FILE__',
        map_file
    )
    yaml_text = yaml_text.replace(
        '__AMCL_MOTION_MODEL__',
        amcl_motion_model
    )
    yaml_text = yaml_text.replace(
        '__MPPI_MOTION_MODEL__',
        mppi_motion_model
    )
    yaml_text = yaml_text.replace(
        '__MAX_VELOCITY__',
        max_velocity
    )
    yaml_text = yaml_text.replace(
        '__MIN_VELOCITY__',
        min_velocity
    )
    yaml_text = yaml_text.replace(
        '__MAX_ACCEL__',
        max_accel
    )
    yaml_text = yaml_text.replace(
        '__MAX_DECEL__',
        max_decel
    )

    tmp_fd, tmp_path = tempfile.mkstemp(
        prefix=f'nav2_params_{ns or "default"}_', suffix='.yaml')
    with os.fdopen(tmp_fd, 'w') as f:
        f.write(yaml_text)

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        #'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        #'docking_server',
    ]

    remappings = [('map', '/map')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=tmp_path,
            root_key=ns,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Node(
            #     package='nav2_route',
            #     executable='route_server',
            #     name='route_server',
            #     namespace=ns,
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Node(
            #     package='opennav_docking',
            #     executable='opennav_docking',
            #     name='docking_server',
            #     namespace=ns,
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=ns,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # ComposableNode(
                    #     package='nav2_route',
                    #     plugin='nav2_route::RouteServer',
                    #     name='route_server',
                    #     namespace=ns,
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        namespace=ns,
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # ComposableNode(
                    #     package='opennav_docking',
                    #     plugin='opennav_docking::DockingServer',
                    #     name='docking_server',
                    #     namespace=ns,
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        namespace=ns,
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    return [load_nodes, load_composable_nodes]


def generate_launch_description():
    navigation_dir = get_package_share_directory('uvot_navigation')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='omni',
        choices=['omni', 'skid', 'diff'],
        description='UVOT locomotion configuration',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigation_dir,'config','nav2_params_common.yaml'),
        description=(
            'Full path to the ROS2 parameters YAML TEMPLATE (__ROBOT_NS__)'
        ),
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map YAML file',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # OpaqueFunction: resuelve 'namespace' a texto plano y construye los
    # nodos (y hace la sustitucion __ROBOT_NS__ del yaml) en tiempo de
    # lanzamiento.
    ld.add_action(OpaqueFunction(function=_build_nodes))

    return ld
