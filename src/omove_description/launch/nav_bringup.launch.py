import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('omove_description')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # This entry always localizes on a saved map. Explicit Python-style
            # False also prevents an inherited 'slam=false' from breaking
            # Humble Nav2's PythonExpression(['not ', slam]).
            'slam': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': 'False',
            'autostart': 'true',
            'map': map_file,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'room.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        nav2,
    ])

