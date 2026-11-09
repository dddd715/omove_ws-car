import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def launch_setup(context):
    pkg_share = get_package_share_directory('omove_description')
    xacro_file = LaunchConfiguration('robot_file').perform(context)
    robot_description = xacro.process_file(xacro_file, mappings={
        'controllers_file': LaunchConfiguration('controllers_file').perform(context),
        'wheel_control_mode': LaunchConfiguration('wheel_control_mode').perform(context),
    }).toxml()

    # 发布TF和URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # 把小车插入Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'omove',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z')],
        output='screen'
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

    return [rsp, gazebo, spawn, kinematics, odometry] + controllers


def generate_launch_description():
    share = get_package_share_directory('omove_description')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('controllers_file', default_value=os.path.join(share, 'config', 'mecanum_controllers.yaml')),
        DeclareLaunchArgument('robot_file', default_value=os.path.join(share, 'urdf', 'omove.urdf.xacro')),
        DeclareLaunchArgument('wheel_control_mode', default_value='velocity', choices=['velocity', 'velocity_pid']),
        OpaqueFunction(function=launch_setup)])

