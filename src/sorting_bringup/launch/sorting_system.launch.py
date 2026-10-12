"""系统级一键启动（方案A链路）：仿真 + MoveIt2 + 点云预处理 + GraspNet + 抓取执行 + 评估。

已砍掉 2D 检测 / 垂直抓取旧链路。运行时链路：
    Gazebo 点云 -> pointcloud_preprocessor -> graspnet_planner_node
        -> /sorting/graspnet_grasps -> graspnet_pick_place_node -> MoveIt 执行

用法：
    ros2 launch sorting_bringup sorting_system.launch.py
    ros2 launch sorting_bringup sorting_system.launch.py gui:=false rviz:=true

启动后另开终端触发分拣：
    ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
        "{use_closed_loop: false, continuous: true}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_bringup.venv_env import venv_pythonpath

# 仅点云预处理 / GraspNet 推理需要 .venv 依赖（scikit-learn / torch）；
# 按节点注入，避免 venv 的 numpy 1.26 污染 MoveIt 等仍依赖旧 numpy 的节点。
VENV_ENV = {'PYTHONPATH': venv_pythonpath()}


def generate_launch_description():
    sim_share = get_package_share_directory('sorting_simulation')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 图形界面'),
        DeclareLaunchArgument('rviz', default_value='false', description='启动 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_bringup_gz.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
    )

    preprocessor = Node(
        package='sorting_perception',
        executable='pointcloud_preprocessor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env=VENV_ENV,
    )

    graspnet_planner = Node(
        package='sorting_grasp_planning',
        executable='graspnet_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env=VENV_ENV,
    )

    pick_place = Node(
        package='sorting_motion_control',
        executable='graspnet_pick_place_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    metrics_logger = Node(
        package='sorting_evaluation',
        executable='metrics_logger_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        arguments
        + [
            simulation,
            move_group,
            preprocessor,
            graspnet_planner,
            pick_place,
            metrics_logger,
        ]
    )