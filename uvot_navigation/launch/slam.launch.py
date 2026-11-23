"""Mapeo con slam_toolbox (online asíncrono) para un robot UVOT.

    ros2 launch uvot_navigation slam.launch.py namespace:=robot1

Usa el /tf global (frames con prefijo <ns>/) y publica el mapa en /map.
Para guardar el mapa al terminar:
    ros2 run nav2_map_server map_saver_cli -f <ruta>/<nombre> \
        --ros-args -p map_subscribe_transient_local:=true
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def _lanzar(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    plantilla = os.path.join(get_package_share_directory('uvot_navigation'),
                             'config', 'slam_toolbox_mapping.yaml')
    texto = open(plantilla).read().replace('__ROBOT_NS__', ns)
    fd, params = tempfile.mkstemp(prefix=f'slam_{ns}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(texto)
    # En Jazzy, slam_toolbox es un nodo con ciclo de vida: hay que configurarlo
    # y activarlo (igual que hace su online_async_launch.py). Con un Node
    # normal se queda parado tras "Node using stack size".
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[params, {'use_sim_time': sim,
                                              'use_lifecycle_manager': False}])
    configurar = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activar = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))
    return [slam, configurar, activar]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_lanzar),
    ])
