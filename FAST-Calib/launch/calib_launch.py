#!/usr/bin/env python3
"""
ROS2 launch file for FAST-Calib single-scene calibration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fast_calib')
    config_path = os.path.join(pkg_dir, 'config', 'qr_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz_cfg', 'fast_livo2.rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[config_path]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz,
        fast_calib_node,
        rviz_node,
    ])
