#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    feat_dir = get_package_share_directory('feature_tracker')
    est_dir = get_package_share_directory('plvins_estimator')

    config_path_default = os.path.join(feat_dir, 'config', 'euroc', 'loop.yaml')
    vins_path_default = '/home/ros/rosws/PL_VINS_ros2_ws/src'

    declare_config = DeclareLaunchArgument('config_path', default_value=config_path_default)
    declare_vins = DeclareLaunchArgument('vins_path', default_value=vins_path_default)

    config_path = LaunchConfiguration('config_path')
    vins_path = LaunchConfiguration('vins_path')

    return LaunchDescription([
        declare_config,
        declare_vins,

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/ros/rosws/PL_VINS_ros2_ws/src/config/vins_rviz_config.rviz"],
            output="screen"
        ),

        Node(package='feature_tracker', executable='feature_tracker',
             name='feature_tracker', output='log',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),

        Node(package='feature_tracker', executable='LineFeature_tracker',
             name='linefeature_tracker', output='log',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),

        Node(package='plvins_estimator', executable='plvins_estimator',
             name='plvins_estimator', output='log',
             parameters=[{'config_file': config_path, 'vins_folder': vins_path}]),

        Node(package='pose_graph', executable='pose_graph',
             name='pose_graph', output='screen',
             parameters=[{'config_file': config_path, 'visualization_shift_x': 0,
                          'visualization_shift_y': 0, 'skip_cnt': 0, 'skip_dis': 0.0}]),

        Node(package='image_node_b', executable='image_node_b',
             name='image_node_b', output='log'),
    ])
