#!/usr/bin/env python3
"""Launch PL-VINS with EuRoC config (fixed extrinsic, no loop closure)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    feat_dir = get_package_share_directory('feature_tracker')
    est_dir = get_package_share_directory('plvins_estimator')

    config_path_default = os.path.join(feat_dir, 'config', 'euroc', 'euroc_config_fix_extrinsic.yaml')
    vins_path_default = '/home/ros/rosws/PL_VINS_ros2_ws/src'

    config_path = LaunchConfiguration('config_path')
    vins_path = LaunchConfiguration('vins_path')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config_path_default),
        DeclareLaunchArgument('vins_path', default_value=vins_path_default),

        Node(package='feature_tracker', executable='feature_tracker',
             name='feature_tracker', output='screen',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),

        Node(package='feature_tracker', executable='LineFeature_tracker',
             name='linefeature_tracker', output='screen',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),

        Node(package='plvins_estimator', executable='plvins_estimator',
             name='plvins_estimator', output='screen',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/ros/rosws/PL_VINS_ros2_ws/src/config/vins_rviz_config.rviz"],
            output="screen"
        )
    ])
