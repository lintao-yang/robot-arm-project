"""仅启动业务算法节点（不含 Gazebo / MoveIt），用于分层调试。

方案A链路（点云 → GraspNet 6DoF → MoveIt 执行），已砍掉 2D 检测/垂直抓取旧链路：
    1. pointcloud_preprocessor   —— 点云预处理（ROI/去桌面/聚类）
    2. graspnet_planner_node     —— GraspNet 6DoF 抓取检测（GPU）
    3. graspnet_pick_place_node  —— 抓取执行（MoveIt 规划 + effort 力控夹爪）
    4. metrics_logger_node       —— 指标记录

分层调试工作流（三个终端）：
    终端1（常驻，舞台）: ros2 launch sorting_simulation sim_bringup_gz.launch.py gui:=true
    终端2（常驻，舞台）: ros2 launch sorting_simulation move_group.launch.py
    终端3（改完就重启）: ros2 launch sorting_bringup sorting_nodes.launch.py

这样 Gazebo 里的机械臂与场景全程保持不动，改完算法代码只需重启终端3，
即可在同一世界中立即验证，无需重启仿真。

注意：终端1 常驻时单独重启本文件，必须保持 use_sim_time:=true，
否则节点使用墙上时钟会与仿真时间不一致。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_bringup.venv_env import venv_pythonpath

# 仅点云预处理 / GraspNet 推理需要 .venv 依赖（scikit-learn / torch）；
# 按节点注入，避免 venv 的 numpy 1.26 污染其它节点（见 venv_env.py 说明）。
VENV_ENV = {'PYTHONPATH': venv_pythonpath()}


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

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
            preprocessor,
            graspnet_planner,
            pick_place,
            metrics_logger,
        ]
    )