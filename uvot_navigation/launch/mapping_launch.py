import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    pkg_share = get_package_share_directory('uvot_navigation')
    params_template = os.path.join(
        pkg_share,
        'config',
        'mapper_params_online_async.yaml'
    )

    # Load the template and replace the robot namespace.
    with open(params_template, 'r') as f:
        params_text = f.read()

    params_text = params_text.replace('__ROBOT_NS__', namespace)

    # Generate a temporary parameter file for this robot instance.
    temp_file = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'uvot_slam_{namespace}_',
        suffix='.yaml',
        delete=False
    )
    temp_file.write(params_text)
    temp_file.close()

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
            temp_file.name,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return [slam_toolbox]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot',
            description='Robot namespace'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        OpaqueFunction(function=launch_setup),
    ])
