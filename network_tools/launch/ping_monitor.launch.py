# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('network_tools')
    default_config = os.path.join(pkg_share, 'config', 'ping_targets.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to YAML config file with ping targets',
        ),
        Node(
            package='network_tools',
            executable='ping_monitor_node',
            name='ping_monitor',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
