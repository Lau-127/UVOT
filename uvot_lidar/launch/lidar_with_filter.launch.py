from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os
import tempfile
from ament_index_python.packages import get_package_share_directory


def _build_actions(context, *args, **kwargs):
    # 'namespace' resuelto a string plano de Python en tiempo de lanzamiento
    # -- mismo valor que uses en robot_state_publisher / odom_to_tf /
    # navigation_launch para este robot.
    ns = LaunchConfiguration('namespace').perform(context)

    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_s2_launch.py'
    )
    serial_port = LaunchConfiguration('serial_port').perform(context)

    # Mismo problema que en odom_to_tf: el nodo queda como
    # '/<ns>/scan_to_scan_filter_chain', pero el YAML de filtros tiene la
    # clave en plano ('scan_to_scan_filter_chain:'). Sin envolverla bajo
    # '<ns>:', ROS2 ignora el archivo entero sin avisar y el filtro de
    # rango se queda con los valores por defecto del paquete, no los tuyos.
    filter_yaml_path = os.path.join(
        get_package_share_directory('uvot_lidar'),
        'config',
        'lidar_range_filter.yaml'
    )
    with open(filter_yaml_path, 'r') as f:
        filter_yaml_text = f.read()
    if ns:
        indented = '\n'.join(
            ('  ' + line if line.strip() else line)
            for line in filter_yaml_text.splitlines()
        )
        filter_yaml_text = f'{ns}:\n{indented}\n'

    tmp_fd, tmp_filter_path = tempfile.mkstemp(
        prefix=f'lidar_range_filter_{ns or "default"}_', suffix='.yaml')
    with os.fdopen(tmp_fd, 'w') as f:
        f.write(filter_yaml_text)

    return [
        GroupAction([
            PushRosNamespace(ns),

            # LIDAR driver: PushRosNamespace prefija automáticamente los
            # TOPICS del nodo (scan -> <ns>/scan), pero NO el contenido
            # del parámetro 'frame_id' (es solo un string) — ese se sigue
            # construyendo a mano con el namespace resuelto.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sllidar_launch_path),
                launch_arguments={
                    'serial_port': serial_port,
                    'frame_id': f'{ns}/base_scan',
                }.items()
            ),

            # Laser filter chain: al estar en el mismo namespace, sus
            # topics por defecto 'scan' / 'scan_filtered' se resuelven
            # automáticamente a '<ns>/scan' / '<ns>/scan_filtered'.
            Node(
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                name='scan_to_scan_filter_chain',
                output='screen',
                parameters=[tmp_filter_path],
            ),
        ]),
    ]


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot3',
        description='Namespace del robot (mismo valor que en el resto de launch de omnibot)'
    )
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lidar',
        description='Puerto serie del lidar'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_serial_port_cmd,
        OpaqueFunction(function=_build_actions),
    ])
