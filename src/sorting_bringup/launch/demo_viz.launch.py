"""演示模式启动：完整分拣系统（headless）+ Web 可视化操作台基础设施。

组成：
  1. sorting_system.launch.py (gui:=false, rviz:=false) —— 完整分拣管线
  2. rosbridge_websocket (ws://<host>:9090)              —— 话题/服务 WebSocket 桥
  3. web_video_server (http://<host>:8080)               —— 相机/检测图 MJPEG 流
  4. urdf_relay                                          —— URDF 重发给浏览器
  5. python http.server (http://<host>:8777)             —— 操作台页面与网格资源

用法：
  ros2 launch sorting_bringup demo_viz.launch.py
  浏览器打开 http://localhost:8777/web_ui/index.html
（WSL2 默认启用 localhost 转发，Windows 侧浏览器可直接访问）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('sorting_bringup')
    rosbridge_share = get_package_share_directory('rosbridge_server')

    # 工作区根目录：优先取 env.sh 注入的 WORKSPACE_DIR，
    # 否则由 install/<pkg>/share/<pkg> 向上 4 级推导。
    fallback = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(bringup_share))))
    workspace = os.environ.get('WORKSPACE_DIR', fallback)

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sorting_system.launch.py')
        ),
        launch_arguments={'gui': 'false', 'rviz': 'false'}.items(),
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{'port': 8080}],
    )

    urdf_relay = Node(
        package='sorting_bringup',
        executable='urdf_relay',
        output='screen',
    )

    web_ui_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8777', '--directory', workspace],
        name='web_ui_server',
        output='screen',
    )

    return LaunchDescription([
        system,
        # 可视化设施错峰启动，避开系统初始化高峰
        TimerAction(period=2.0, actions=[web_ui_server]),
        TimerAction(period=5.0, actions=[rosbridge, video_server]),
        TimerAction(period=6.0, actions=[urdf_relay]),
    ])
