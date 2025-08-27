# 文件位置建議：src/main-pkg/launch/bringup.launch.py  (或新建一個 bringup 套件也行)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 可調整的延遲（秒）：先啟 realsense，再等 N 秒啟 opencv，再等 M 秒啟 main-pkg
    opencv_delay = LaunchConfiguration('opencv_delay')
    main_delay   = LaunchConfiguration('main_delay')

    declare_opencv_delay = DeclareLaunchArgument('opencv_delay', default_value='3.0')
    declare_main_delay   = DeclareLaunchArgument('main_delay', default_value='7.0')

    # 1) 先啟 RealSense 官方的 rs_launch.py
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'
        ]),
        launch_arguments={
            # 依你 log 使用的名稱，讓 topic 走 /latuuu_camera/...
            'camera_name': 'latuuu_camera',
            # 需要的話可在這裡加 profile，例如：
            # 'depth_module.profile': '640x480x15',
            # 'rgb_camera.profile':   '640x480x15',
            # 'initial_reset': 'true',
        }.items()
    )

    # 2) 第二階段：OpenCV 節點
    opencv_group = GroupAction([
        Node(package='opencv_ros', executable='coffee', output='screen'),
        Node(package='opencv_ros', executable='table',  output='screen'),
    ])

    # 3) 第三階段：你的主程式群
    main_group = GroupAction([
        Node(package='main-pkg', executable='SimLower',     output='screen'),
        Node(package='main-pkg', executable='ChassisPilot', output='screen'),
        Node(package='main-pkg', executable='MissionTwo',   output='screen'),
    ])

    return LaunchDescription([
        declare_opencv_delay,
        declare_main_delay,
        rs_launch,
        # 等 realsense 起來後再啟 opencv 與 main（用簡單定時器避免搶 /dev/video 與參數尚未套用）
        TimerAction(period=opencv_delay, actions=[opencv_group]),
        TimerAction(period=main_delay,   actions=[main_group]),
    ])
