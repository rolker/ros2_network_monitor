from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mikrotik_monitor')
    default_config = os.path.join(pkg_share, 'config', 'mikrotik_monitor.yaml')

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
            package='mikrotik_monitor',
            executable='mikrotik_monitor_node',
            name='mikrotik_monitor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
