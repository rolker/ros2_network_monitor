from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('teltonika_monitor')
    default_config = os.path.join(
        pkg_share, 'config', 'teltonika_monitor.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the config YAML file',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Node namespace',
        ),
        Node(
            package='teltonika_monitor',
            executable='teltonika_monitor_node',
            name='teltonika_monitor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
