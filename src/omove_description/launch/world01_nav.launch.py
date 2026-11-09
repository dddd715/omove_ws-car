"""Nav2 in world01; require an explicit map rather than loading the room map."""
import os
import socket
from urllib.parse import urlparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def require_free_gazebo_master(context):
    """Fail before starting any robot/Nav2 nodes if an old simulation survives."""
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345'))
    try:
        connection = socket.create_connection((uri.hostname or 'localhost', uri.port or 11345), timeout=1.)
    except ConnectionRefusedError:
        return []
    except OSError as error:
        raise RuntimeError(f'Cannot verify Gazebo master is free: {error}') from error
    connection.close()
    raise RuntimeError(
        'Gazebo master is already running. Stop the old OMOVE launch and its '
        'remaining gzserver/robot nodes before starting world01_nav; otherwise '
        'duplicate odometry and TF publishers can corrupt navigation.')


def generate_launch_description():
    share = get_package_share_directory('omove_description')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a saved world01 map YAML'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-1.0'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=require_free_gazebo_master),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'world01_sim.launch.py')),
            launch_arguments={'slam': 'False', 'gui': LaunchConfiguration('gui'),
                              'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'nav_bringup.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': 'true'}.items()),
    ])
