#!/usr/bin/env python3
"""Publish benchmark ground truth trajectory for evaluation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    seq_name = LaunchConfiguration('sequence_name', default='MH_05_difficult')
    data_path = os.path.join(get_package_share_directory('benchmark_publisher'),
                             'config', LaunchConfiguration('sequence_name').perform, 'data.csv')

    return LaunchDescription([
        DeclareLaunchArgument('sequence_name', default_value='MH_05_difficult'),
        Node(package='benchmark_publisher', executable='benchmark_publisher',
             name='benchmark_publisher', output='screen',
             parameters=[{'data_name': data_path}],
             remappings=[('estimated_odometry', '/plvins_estimator/odometry')]),
    ])
