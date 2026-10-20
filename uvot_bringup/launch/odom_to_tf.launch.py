#!/usr/bin/env python3
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_node(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)

    config_file_path = os.path.join(
        get_package_share_directory("uvot_bringup"), "config", "odom_to_tf.yaml"
    )
    with open(config_file_path, 'r') as f:
        yaml_text = f.read()
    yaml_text = yaml_text.replace('__ROBOT_NS__', ns)

    # IMPORTANTE: el nodo queda como '/<ns>/odom_to_tf' (namespaced), asi
    # que la clave de nivel superior del YAML tiene que reflejar el
    # namespace o ROS2 ignora el archivo entero sin avisar. Envolvemos el
    # contenido original bajo '<ns>:' en vez de dejarlo en plano.
    if ns:
        indented = '\n'.join(
            ('  ' + line if line.strip() else line)
            for line in yaml_text.splitlines()
        )
        yaml_text = f'{ns}:\n{indented}\n'

    tmp_fd, tmp_path = tempfile.mkstemp(
        prefix=f'odom_to_tf_{ns or "default"}_', suffix='.yaml')
    with os.fdopen(tmp_fd, 'w') as f:
        f.write(yaml_text)

    return [
        Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_to_tf",
            namespace=ns,
            output="screen",
            parameters=[tmp_path],
            #remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
    ]


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace (same value in robot_state_publisher.launch.py and navigation_launch.py)'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        OpaqueFunction(function=_build_node),
    ])
