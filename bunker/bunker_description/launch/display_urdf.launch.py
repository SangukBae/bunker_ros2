#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import FindExecutable

def generate_launch_description():
    pkg_desc = FindPackageShare('bunker_description')
    urdf_file_default = PathJoinSubstitution([pkg_desc, 'urdf', 'bunker.urdf.xacro'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path    = LaunchConfiguration('urdf')
    rviz         = LaunchConfiguration('rviz')
    rviz_cfg     = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_urdf         = DeclareLaunchArgument('urdf', default_value=urdf_file_default,
                                                 description='Path to URDF/Xacro (processed by xacro)')
    declare_rviz         = DeclareLaunchArgument('rviz', default_value='true')
    declare_rviz_cfg     = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_desc, 'rviz', 'display.rviz'])
    )

    # xacro를 통해 urdf/xacro 모두 처리
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_path])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_urdf,
        declare_rviz,
        declare_rviz_cfg,
        rsp,
        rviz_node
    ])
