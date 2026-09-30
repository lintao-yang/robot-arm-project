"""启动 MoveIt2 move_group（复用官方 Panda 配置，但使用本项目的 URDF/SRDF）。

关键差异：
    * URDF 使用本项目 panda_gz.urdf.xacro（含惯量与 world 固定底座）
    * SRDF 由官方 panda.srdf 派生，但把 virtual_joint 由 floating 改为 fixed，
      因为 URDF 中已通过 panda_base_mount 把底座固定到 world
    * 运动学 / 关节限速 / OMPL / 控制器配置直接复用官方 YAML
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_simulation.urdf_utils import load_robot_description


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


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

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    parameters = {
        'robot_description': robot_description,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': _load_yaml(
            os.path.join(panda_config_dir, 'kinematics.yaml')
        ),
        'robot_description_planning': _load_yaml(
            os.path.join(panda_config_dir, 'joint_limits.yaml')
        ),
        'planning_pipelines': ['ompl'],
        'ompl': _load_yaml(os.path.join(panda_config_dir, 'ompl_planning.yaml')),
        'use_sim_time': use_sim_time,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'allow_trajectory_execution': True,
        'monitor_dynamics': False,
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
            'robot_description_kinematics': _load_yaml(
                os.path.join(panda_config_dir, 'kinematics.yaml')
            ),
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz2 可视化'),
        move_group,
        rviz_node,
    ])