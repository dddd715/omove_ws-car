import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('omove_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'omove.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 1. robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Gazebo
    world_file = os.path.join(pkg_share, 'worlds', 'room.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 3. 把小车插进Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'omove',
                   '-x', '1.5', '-y', '0', '-z', '0.1'],
        output='screen'
    )


    # 5. slam_toolbox建图
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_params.yaml'),
        }.items()
    )

    # Spawners wait for controller_manager, which is created by the Gazebo plugin.
    controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'],
            output='screen',
        )
        for controller in [
            'joint_state_broadcaster',
            'front_left_velocity_controller',
            'front_right_velocity_controller',
            'rear_left_velocity_controller',
            'rear_right_velocity_controller',
        ]
    ]
    kinematics = Node(
        package='omove_description',
        executable='mecanum_kinematics.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    odometry = Node(
        package='omove_description',
        executable='mecanum_odometry.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([rsp, gazebo, spawn, slam, kinematics, odometry] + controllers)

