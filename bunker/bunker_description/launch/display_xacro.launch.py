# bunker_description/launch/display_xacro.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_gui = LaunchConfiguration("gui", default="true")         # joint_state_publisher_gui on/off
    run_rviz = LaunchConfiguration("rviz", default="true")       # rviz2 on/off

    # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("bunker_description"),
        "urdf",
        "bunker.xacro",
    ])

    # Run xacro and capture the output as a string parameter
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    # joint_state_publisher_gui (for interactive sliders)
    jsp_gui = Node(
        condition=IfCondition(use_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # joint_state_publisher (headless fallback)
    jsp = Node(
        condition=UnlessCondition(use_gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            # GUI 없이도 기본 각도를 퍼블리시하도록 하면 RobotModel이 보입니다.
            {"publish_default_positions": True},
        ],
        output="screen",
    )

    # rviz2 (no config; 기본 화면으로 실행)
    rviz = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # RViz 설정 파일이 있다면 아래처럼 추가:
        # arguments=["-d", PathJoinSubstitution([FindPackageShare("bunker_description"), "rviz", "bunker.rviz"])],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        rsp,
        jsp_gui,
        jsp,
        rviz,
    ])
