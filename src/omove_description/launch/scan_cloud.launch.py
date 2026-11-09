"""Optional scan projection + RViz; run beside an existing robot/Nav2 launch."""
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
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='omove_description', executable='scan_to_cloud.py',
             parameters=[{'use_sim_time': sim_time,
                          'target_frame': LaunchConfiguration('target_frame')}],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'omove_pointcloud.rviz')],
             parameters=[{'use_sim_time': sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
