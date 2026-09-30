"""仿真环境启动（Gazebo Sim / Ignition Fortress 版）。

与 Classic 版 sim_bringup.launch.py 的对应关系：
    gazebo_ros 的 gazebo.launch.py   -> ros_gz_sim 的 gz_sim.launch.py
    spawn_entity.py（/spawn_entity）  -> ros_gz_sim create（/world/<w>/create）
    gazebo_ros_camera 插件            -> ignition-gazebo-sensors-system + ros_gz_bridge
    gazebo_ros2_control               -> gz_ros2_control（URDF 内已配置）

用法：
    ros2 launch sorting_simulation sim_bringup_gz.launch.py
    ros2 launch sorting_simulation sim_bringup_gz.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_simulation.urdf_utils import load_robot_description

# Gazebo 话题 -> ROS 话题（相机、物体实时位姿）
# 注意：仿真时钟单独处理（见下面的 clock_bridge）——
#   gz 侧的 /clock 话题存在但**不发布**，真正发布的是 /world/<world>/clock；
#   而 rcl 的 use_sim_time 只认 /clock，因此需要桥接 + 重映射。
BRIDGE_TOPICS = [
    '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
    '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
    '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    '/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    # 物体实时位姿（gz Pose_V -> tf2 TFMessage），scene_manager 用它发布真值
    '/world/sorting_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
]


def generate_launch_description():
    sim_share = get_package_share_directory('sorting_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(sim_share, 'worlds', 'sorting_world_sim.sdf')
    xacro_file = os.path.join(sim_share, 'urdf', 'panda_gz.urdf.xacro')

    # xacro 展开 + 注入惯量参数 + 手指碰撞体简化（与 Classic 版共用同一处理链）
    robot_description = load_robot_description(xacro_file)

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo Sim 图形界面'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用仿真时钟'),
    ]

    # -r：加载后立即运行（不暂停）；gui:=false 时仅起服务端（headless）
    # 注意：LaunchConfiguration 对象在 Python 中恒为 truthy，不能直接用 if gui 判断，
    # 必须用 IfCondition/UnlessCondition 让 launch 在运行时按实参选择。
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v3 {world_file}'}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v3 -s {world_file}'}.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # 机器人生成：从 /robot_description 读取并创建名为 panda 的模型
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'panda',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-allow_renaming', 'false',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=BRIDGE_TOPICS,
        # Gazebo Sim 相机的话题名与 Classic 版插件不同，这里重映射回项目既定话题名，
        # 使视觉/定位节点无需改动（system.yaml 的 topics 段保持有效）。
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 仿真时钟：桥接 gz 的世界时钟并重映射到 /clock（rcl 只认这个名字）
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            f'/world/sorting_world/clock'
            '@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        remappings=[('/world/sorting_world/clock', '/clock')],
    )

    def controller_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '90'],
        )

    scene_manager = Node(
        package='sorting_simulation',
        executable='scene_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_gui,
            gazebo_headless,
            robot_state_publisher,
            bridge,
            clock_bridge,
            # 等 Gazebo Sim 服务端就绪后再生成机器人与控制器
            TimerAction(period=6.0, actions=[spawn_robot]),
            TimerAction(period=12.0, actions=[
                controller_spawner('joint_state_broadcaster'),
            ]),
            TimerAction(period=14.0, actions=[
                controller_spawner('panda_arm_controller'),
                controller_spawner('panda_hand_controller'),
            ]),
            TimerAction(period=18.0, actions=[scene_manager]),
        ]
    )