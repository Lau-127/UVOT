"""Nav2 de UVOT para un robot, con TF global y frames con prefijo <ns>/.

    ros2 launch uvot_navigation nav.launch.py namespace:=robot1 robot_type:=omni \
        map:=/home/robot/maps/lab.yaml

Lanza, dentro del namespace del robot:
  - localización (si fase:=amcl): map_server + amcl
  - navegación: controller, smoother, planner, behaviors, bt_navigator,
    waypoint_follower, velocity_smoother y collision_monitor

Es una copia reducida de localization_launch.py y navigation_launch.py de
nav2_bringup (Jazzy) SIN el remap /tf -> tf: todos los robots comparten el
/tf global y sus frames no chocan porque llevan el prefijo robotX/. Así un
solo RViz ve todos los robots sobre el mismo mapa.

El YAML de parámetros se genera al lanzar (uvot_nav2_params.py) a partir de
config/nav2_params_common.yaml, según robot_type y fase.

Cadena de velocidades:
  controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
  -> collision_monitor -> /<ns>/cmd_vel (firmware)
"""
import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uvot_nav2_params import generar  # noqa: E402


def _lanzar(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    robot = LaunchConfiguration('robot_type').perform(context)
    fase = LaunchConfiguration('fase').perform(context)
    mapa = LaunchConfiguration('map').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    log = LaunchConfiguration('log_level').perform(context)
    gc = LaunchConfiguration('goal_checker').perform(context)
    xy_tol = float(LaunchConfiguration('xy_tol').perform(context))
    yaw_tol = float(LaunchConfiguration('yaw_tol').perform(context))

    ajustes = LaunchConfiguration('ajustes').perform(context)

    texto = generar(robot, fase, ns, mapa, goal_checker=gc, xy_tol=xy_tol, yaw_tol=yaw_tol,
                    ajustes=ajustes)
    if ajustes:
        print(f'[nav.launch] ajustes de FollowPath: {ajustes}')
    # Los nodos quedan como /<ns>/<nodo>: las claves del YAML tienen que ir
    # bajo '<ns>:' o ROS 2 ignora los parámetros sin avisar (igual que en
    # odom_to_tf.launch.py; nav2_bringup lo hace con RewrittenYaml root_key).
    if ns:
        texto = f'{ns}:\n' + '\n'.join(('  ' + l if l.strip() else l)
                                         for l in texto.splitlines()) + '\n'
    fd, params = tempfile.mkstemp(prefix=f'nav2_{ns}_{robot}_{fase}_{gc}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(texto)
    print(f'[nav.launch] parámetros de Nav2: {params}')

    comunes = dict(namespace=ns, output='screen',
                   parameters=[params, {'use_sim_time': sim}],
                   arguments=['--ros-args', '--log-level', log])
    nodos = []

    # ---------------------------------------------------------- localización
    if fase == 'amcl':
        nodos += [
            # El mapa se publica en /map (global), que es donde lo esperan AMCL
            # y el global costmap (map_topic: /map). Sin el remap saldría en
            # /<ns>/map y AMCL se quedaría en 'Waiting for map...'.
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 remappings=[('map', '/map')], **comunes),
            Node(package='nav2_amcl', executable='amcl', name='amcl', **comunes),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': sim, 'autostart': True,
                              'node_names': ['map_server', 'amcl']}]),
        ]

    # ---------------------------------------------------------- navegación
    nav = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
           'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']
    nodos += [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', remappings=[('cmd_vel', 'cmd_vel_nav')], **comunes),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', **comunes),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **comunes),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', remappings=[('cmd_vel', 'cmd_vel_nav')], **comunes),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **comunes),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **comunes),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', remappings=[('cmd_vel', 'cmd_vel_nav')], **comunes),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', **comunes),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': nav}]),
    ]
    return nodos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('robot_type', default_value='omni',
                              choices=['omni', 'skid', 'diff']),
        DeclareLaunchArgument('fase', default_value='amcl',
                              choices=['odom', 'firmware', 'amcl'],
                              description='amcl: con mapa | firmware/odom: sin mapa'),
        DeclareLaunchArgument('map', default_value='',
                              description='Ruta al .yaml del mapa (fase amcl)'),
        DeclareLaunchArgument('goal_checker', default_value='general',
                              choices=['general', 'precise'],
                              description='general: 0.35 m / 0.5 rad | precise: xy_tol / '
                                          'yaw_tol (ajuste del MPPI y ensayos de validación)'),
        DeclareLaunchArgument('xy_tol', default_value='0.05',
                              description='Tolerancia xy del goal checker precise (m)'),
        DeclareLaunchArgument('yaw_tol', default_value='0.10',
                              description='Tolerancia de orientación del goal checker precise (rad)'),
        DeclareLaunchArgument('ajustes', default_value='',
                              description="Sobrescribe parámetros de FollowPath, p. ej. "
                                          "'TwirlingCritic.twirling_cost_weight=40,vx_std=0.3'"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_lanzar),
    ])