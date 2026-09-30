"""系统级一键启动：仿真 + MoveIt2 + 视觉 + 抓取规划 + 运动执行 + 任务调度 + 评估。

用法：
    ros2 launch sorting_bringup sorting_system.launch.py
    ros2 launch sorting_bringup sorting_system.launch.py gui:=false rviz:=true
    ros2 launch sorting_bringup sorting_system.launch.py vision_backend:=classical

启动后另开终端触发分拣：
    ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
        "{use_closed_loop: true, continuous: true}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('sorting_simulation')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vision_backend = LaunchConfiguration('vision_backend')
    model_path = LaunchConfiguration('model_path')

    arguments = [
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 图形界面'),
        DeclareLaunchArgument('rviz', default_value='false', description='启动 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('vision_backend', default_value='yolo',
                              description='视觉后端: yolo | classical'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='YOLO 权重路径（空则回退到颜色分割）'),
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

    detector = Node(
        package='sorting_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'backend': vision_backend,
            'model_path': model_path,
        }],
    )

    pose_estimator = Node(
        package='sorting_pose_estimation',
        executable='pose_estimator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    grasp_planner = Node(
        package='sorting_grasp_planning',
        executable='grasp_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    motion_executor = Node(
        package='sorting_motion_control',
        executable='motion_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    task_planner = Node(
        package='sorting_task_planning',
        executable='task_planner_node',
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
            detector,
            pose_estimator,
            grasp_planner,
            motion_executor,
            task_planner,
            metrics_logger,
        ]
    )