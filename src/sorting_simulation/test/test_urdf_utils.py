"""URDF 处理链（xacro 展开 + 惯量注入 + 手指碰撞体简化）的回归测试。

需在已 source ROS 与工作空间的环境运行（依赖 ament 索引与 xacro）。
"""

import re
from pathlib import Path

XACRO_PATH = (
    Path(__file__).resolve().parents[1] / 'urdf' / 'panda_gz.urdf.xacro'
)


def _urdf():
    from sorting_simulation.urdf_utils import load_robot_description

    return load_robot_description(str(XACRO_PATH))


def test_xacro_expands_and_uses_gz_sim_plugin():
    urdf = _urdf()
    assert '<robot' in urdf and 'name="panda"' in urdf
    assert 'gz_ros2_control/GazeboSimSystem' in urdf
    assert 'gazebo_ros2_control' not in urdf


def test_inertials_injected():
    urdf = _urdf()
    # 所有 link 都应有非零惯量（Gazebo Sim 下零惯量会导致动力学异常）
    for link_name in re.findall(r'<link name="([^"]+)"', urdf):
        assert link_name, '存在无名 link'
    assert urdf.count('<inertial>') >= 10
    assert 'ixx="0"' not in urdf


def test_finger_mimic_removed():
    urdf = _urdf()
    # 只匹配机器人本体 joint（带 type 属性），排除 ros2_control 的接口声明块
    finger_blocks = re.findall(
        r'<joint name="panda_finger_joint\d" type="[^"]*".*?</joint>', urdf, re.S
    )
    assert len(finger_blocks) == 2
    for block in finger_blocks:
        assert '<mimic' not in block
