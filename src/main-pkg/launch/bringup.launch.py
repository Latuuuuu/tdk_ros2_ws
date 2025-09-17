# src/main-pkg/launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    delay = LaunchConfiguration('delay')
    declare_delay = DeclareLaunchArgument('delay', default_value='3.0')

    opencv_group = GroupAction([
        Node(package='opencv_ros', executable='coffee', output='screen'),
        Node(package='opencv_ros', executable='table',  output='screen'),
    ])

    main_group = GroupAction([
        Node(package='main-pkg', executable='MissionManager', output='screen'),
        Node(package='main-pkg', executable='SimLower',     output='screen'),
        Node(package='main-pkg', executable='ChassisPilot', output='screen'),
        Node(package='main-pkg', executable='MissionTwo',   output='screen'),
        Node(package='main-pkg', executable='MissionFour',  output='screen'),

    ])

    return LaunchDescription([
        declare_delay,
        TimerAction(period=delay, actions=[opencv_group]),
        TimerAction(period=delay, actions=[main_group]),
    ])
