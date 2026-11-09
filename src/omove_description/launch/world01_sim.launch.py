"""OMOVE in the bundled world01 scene, with optional SLAM Toolbox."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('omove_description')
    scene = os.path.join(share, 'worlds', 'world01')
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(scene, 'worlds', 'playground.world')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(share, 'config', 'slam_params.yaml')),
        DeclareLaunchArgument('controllers_file', default_value=os.path.join(share, 'config', 'mecanum_controllers.yaml')),
        DeclareLaunchArgument('robot_file', default_value=os.path.join(share, 'urdf', 'omove.urdf.xacro')),
        DeclareLaunchArgument('wheel_control_mode', default_value='velocity_pid', choices=['velocity', 'velocity_pid']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-1.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', [
            os.path.join(scene, 'models'), os.pathsep,
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', [
            scene, os.pathsep,
            # Setting this variable masks Gazebo's built-in fallback. Keep its
            # media/RTShaderLib directory even in a freshly sourced ROS shell.
            '/usr/share/gazebo-11', os.pathsep,
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
                'z': '0.1', 'gui': LaunchConfiguration('gui'),
                'controllers_file': LaunchConfiguration('controllers_file'),
                'robot_file': LaunchConfiguration('robot_file'),
                'wheel_control_mode': LaunchConfiguration('wheel_control_mode'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),
    ])
