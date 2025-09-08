#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc   = FindPackageShare('bunker_description')
    pkg_sim    = FindPackageShare('bunker_gazebo_sim')
    pkg_gz     = FindPackageShare('gazebo_ros')

    default_xacro = PathJoinSubstitution([pkg_desc, 'urdf', 'bunker.xacro'])
    default_world = PathJoinSubstitution([pkg_gz, 'worlds', 'empty.world'])
    default_ctrl  = PathJoinSubstitution([pkg_sim, 'config', 'controllers.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')
    model        = LaunchConfiguration('model')
    robot_name   = LaunchConfiguration('robot_name')
    x, y, z      = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    R, P, Yaw    = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')
    controllers  = LaunchConfiguration('controllers_file')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world        = DeclareLaunchArgument('world', default_value=default_world)
    declare_model        = DeclareLaunchArgument('model', default_value=default_xacro,
                                                description='bunker.xacro path')
    declare_robot_name   = DeclareLaunchArgument('robot_name', default_value='bunker')
    declare_xyz          = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
    ]
    declare_rpy          = [
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
    ]
    declare_controllers  = DeclareLaunchArgument('controllers_file', default_value=default_ctrl,
                                                description='ros2_control controllers yaml')

    # Process xacro -> robot_description
    robot_description = Command([FindExecutable(name='xacro'), ' ', model])

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}]
    )

    # Gazebo (classic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gz, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world}.items()
    )

    # Spawn entity from /robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Yaw
        ],
        output='screen'
    )

    # Controllers: joint_state_broadcaster, diff_drive
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers],
        output='screen'
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers],
        output='screen'
    )

    # 조금 기다렸다 컨트롤러 스폰(가제보/스폰 완료 후)
    delayed_jsb  = TimerAction(period=3.0, actions=[jsb_spawner])
    delayed_diff = TimerAction(period=4.0, actions=[diff_spawner])

    return LaunchDescription(
        [declare_use_sim_time, declare_world, declare_model, declare_robot_name,
         *declare_xyz, *declare_rpy, declare_controllers,
         gazebo, rsp, spawn_entity, delayed_jsb, delayed_diff]
    )
