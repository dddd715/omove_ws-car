"""Bounded multi-scan cloud assembly beside an existing robot simulation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('omove_description')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('window_seconds', default_value='30.0'),
        DeclareLaunchArgument('voxel_size', default_value='0.03'),
        DeclareLaunchArgument('save_path', default_value='/tmp/omove_accumulated.pcd'),
        Node(package='omove_description', executable='scan_to_cloud.py',
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool),
                 'target_frame': 'odom'}], output='screen'),
        Node(package='omove_description', executable='cloud_accumulator.py',
             parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'window_seconds': ParameterValue(LaunchConfiguration('window_seconds'), value_type=float),
                 'voxel_size': ParameterValue(LaunchConfiguration('voxel_size'), value_type=float),
                 'save_path': LaunchConfiguration('save_path')}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'omove_accumulated_cloud.rviz')],
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
