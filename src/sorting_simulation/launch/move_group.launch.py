"""启动 MoveIt2 move_group（复用官方 Panda 配置，但使用本项目的 URDF/SRDF）。

关键差异：
    * URDF 使用本项目 panda_gz.urdf.xacro（含惯量与 world 固定底座）
    * SRDF 由官方 panda.srdf 派生，但把 virtual_joint 由 floating 改为 fixed，
      因为 URDF 中已通过 panda_base_mount 把底座固定到 world
    * 运动学 / 关节限速 / OMPL / 控制器配置直接复用官方 YAML
"""

import glob
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_simulation.urdf_utils import load_robot_description


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _plugin_available(lib_name: str) -> bool:
    """检测 MoveIt 运动学插件 .so 是否存在。"""
    return bool(glob.glob(f'/opt/ros/*/lib/{lib_name}'))


def _kinematics(panda_config_dir):
    """返回 (kinematics 配置, 求解器名称)。

    KDL 是 MoveIt 默认求解器，对本项目"接近垂直、易触及关节限位"的抓取姿态解算成功率偏低。
    按可用性择优（TRAC-IK > pick_ik > KDL 回退），不改变系统的可用性下限。
    注：Humble 官方源无 TRAC-IK 包，实测可用的是 pick_ik。
    """
    kinematics = _load_yaml(os.path.join(panda_config_dir, 'kinematics.yaml'))
    if _plugin_available('libtrac_ik_kinematics_plugin.so'):
        solver, name = 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin', 'TRAC-IK'
        extra = {'kinematics_solver_timeout': 0.05, 'solve_type': 'Distance'}
    elif _plugin_available('libpick_ik_plugin.so'):
        solver, name = 'pick_ik/PickIkPlugin', 'pick_ik'
        # mode=global：允许初始猜测离目标很远。默认/此前使用的 local 只在种子附近搜索，
        # 而 OMPL 目标采样时种子常远离目标位姿（如从 home 去够 z=0.24、倾斜 26° 的抓取位姿），
        # 会导致间歇性的 "Unable to sample any valid states for goal tree"（错误码 99999）。
        # 证据：同一目标位姿用当前状态作种子解不出，换 ±0.3/±0.8rad 抖动种子即解出（2/5、4/5）。
        extra = {'mode': 'global',
                 'position_threshold': 0.001,
                 'orientation_threshold': 0.01,
                 'stop_optimization_on_valid_solution': True}
    else:
        return kinematics, 'KDL（未检测到 TRAC-IK / pick_ik 插件）'
    for group in kinematics.values():
        if isinstance(group, dict):
            group['kinematics_solver'] = solver
            for key, value in extra.items():
                group.setdefault(key, value)
    return kinematics, name


def generate_launch_description():
    sim_share = get_package_share_directory('sorting_simulation')
    panda_config_share = get_package_share_directory('moveit_resources_panda_moveit_config')
    panda_config_dir = os.path.join(panda_config_share, 'config')

    xacro_file = os.path.join(sim_share, 'urdf', 'panda_gz.urdf.xacro')
    robot_description = load_robot_description(xacro_file)

    # SRDF：把虚拟关节改为 fixed，与 URDF 中的 world 固定底座保持一致
    with open(os.path.join(panda_config_dir, 'panda.srdf'), 'r', encoding='utf-8') as handle:
        srdf = handle.read()
    srdf = srdf.replace(
        '<virtual_joint child_link="panda_link0" name="virtual_joint" '
        'parent_frame="world" type="floating"/>',
        '<virtual_joint child_link="panda_link0" name="virtual_joint" '
        'parent_frame="world" type="fixed"/>',
    )
    # V 型槽指尖垫（panda_*finger_pad）与宿主手指由 fixed joint 直连、物理上永不
    # 相对运动；垫片背部与指盒/手掌网格存在毫米级嵌入（附着件的常规设计），
    # 且两垫在手指全闭位（spawn 初始姿态）互相穿透数毫米——这些对都必须豁免
    # 自碰撞检查，否则规划器判定初始状态恒定自碰撞（99999）。
    # 注意：实际抓取规划状态中手指恒 ≥0.019（夹物）或 0.038（张开），
    # 两垫间距 ≥30mm，豁免不影响真实碰撞防护。
    srdf = srdf.replace(
        '</robot>',
        '  <disable_collisions link1="panda_leftfinger" '
        'link2="panda_leftfinger_pad" reason="AttachedPad"/>\n'
        '  <disable_collisions link1="panda_rightfinger" '
        'link2="panda_rightfinger_pad" reason="AttachedPad"/>\n'
        '  <disable_collisions link1="panda_hand" '
        'link2="panda_leftfinger_pad" reason="AttachedPad"/>\n'
        '  <disable_collisions link1="panda_hand" '
        'link2="panda_rightfinger_pad" reason="AttachedPad"/>\n'
        '  <disable_collisions link1="panda_leftfinger_pad" '
        'link2="panda_rightfinger_pad" reason="ClosedPoseOverlap"/>\n'
        '</robot>',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    kinematics, solver_name = _kinematics(panda_config_dir)

    parameters = {
        'robot_description': robot_description,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': kinematics,
        'robot_description_planning': _load_yaml(
            os.path.join(panda_config_dir, 'joint_limits.yaml')
        ),
        'planning_pipelines': ['ompl'],
        # 显式声明默认流水线，否则 move_group 启动时告警 "Using default pipeline 'ompl'"
        'default_planning_pipeline': 'ompl',
        'ompl': _load_yaml(os.path.join(panda_config_dir, 'ompl_planning.yaml')),
        'use_sim_time': use_sim_time,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'allow_trajectory_execution': True,
        'monitor_dynamics': False,
        # ---- octomap 占用图：本项目不使用，显式声明以消除启动报错 ----
        # move_group 总会创建 octomap 占用图监视器，并在缺少配置时报：
        #   [ERROR] No 3D sensor plugin(s) defined for octomap updates
        #   [WARN]  Resolution not specified for Octomap. Assuming resolution = 0.1
        # 本项目的碰撞几何来自 scene_manager 发布的真值物体 + graspnet_pick_place_node 注册的
        # 桌面/分拣箱（经 planning_scene_world），不依赖 octomap。因此显式声明：
        #   * octomap_resolution：给出分辨率，消除 WARN
        #   * sensors：提供一个传感器项，但其 sensor_plugin 以 '~' 开头
        #     —— 这是 MoveIt 的"禁用该更新器"约定，监视器随即空转、无更新器，
        #        不改变任何规划行为（与不使用 octomap 的现状完全一致）。
        'octomap_resolution': 0.1,
        'sensors': ['octomap_monitor_disabled'],
        'octomap_monitor_disabled': {'sensor_plugin': '~'},
    }
    parameters.update(_load_yaml(os.path.join(panda_config_dir, 'moveit_controllers.yaml')))

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[parameters],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(panda_config_share, 'launch', 'moveit.rviz')],
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': srdf,
            'robot_description_kinematics': kinematics,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz2 可视化'),
        LogInfo(msg=f'[move_group] IK 求解器: {solver_name}'),
        move_group,
        rviz_node,
    ])