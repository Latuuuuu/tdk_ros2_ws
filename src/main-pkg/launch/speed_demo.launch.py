from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('main-pkg')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'speed_pub_demo.rviz')

    # 可調參數
    period_ms = LaunchConfiguration('period_ms', default='50')
    state_topic = LaunchConfiguration('state_topic', default='/robot/pose')
    goal_topic  = LaunchConfiguration('goal_topic',  default='/goal')
    use_goal_orientation = LaunchConfiguration('use_goal_orientation', default='false')
    turn_in_place_before_move = LaunchConfiguration('turn_in_place_before_move', default='false')
    rotate_only = LaunchConfiguration('rotate_only', default='false')
    viz_enable = LaunchConfiguration('viz_enable', default='true')
    viz_frame  = LaunchConfiguration('viz_frame',  default='odom')

    return LaunchDescription([
        DeclareLaunchArgument('period_ms', default_value='50'),
        DeclareLaunchArgument('state_topic', default_value='/robot/pose'),
        DeclareLaunchArgument('goal_topic', default_value='/goal'),
        DeclareLaunchArgument('use_goal_orientation', default_value='false'),
        DeclareLaunchArgument('turn_in_place_before_move', default_value='false'),
        DeclareLaunchArgument('rotate_only', default_value='false'),
        DeclareLaunchArgument('viz_enable', default_value='true'),
        DeclareLaunchArgument('viz_frame', default_value='odom'),

        #1) SpeedPublisher
        Node(
            package='main-pkg',
            executable='SpeedPublisher',
            name='speed_publisher',
            output='screen',
            parameters=[{
                'period_ms': LaunchConfiguration('period_ms'),
                'state_topic': LaunchConfiguration('state_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'use_goal_orientation': LaunchConfiguration('use_goal_orientation'),
                'turn_in_place_before_move': LaunchConfiguration('turn_in_place_before_move'),
                'rotate_only': LaunchConfiguration('rotate_only'),
                'viz_enable': LaunchConfiguration('viz_enable'),
                'viz_frame': LaunchConfiguration('viz_frame'),
                'use_sim_time': True,
                # 其他控制參數也可在這裡加
            }],
            remappings=[
                # 如果要改指令 topic，可在這裡 remap
                # ('/robot/cmd_vel', '/robot/cmd_vel')
            ],
        ),

        # 2) 模擬下位機（你的 SimLower 範例）
        Node(
            package='main-pkg',
            executable='SimLower',
            name='sim_lower',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                # 確保吃 /robot/cmd_vel，吐 /robot/pose
                # ('/cmd_vel', '/robot/cmd_vel'),
                # ('/odom', '/robot/pose'),
            ],
        ),

        # # 3) RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_cfg],
        #     output='screen',
        # ),
    ])
