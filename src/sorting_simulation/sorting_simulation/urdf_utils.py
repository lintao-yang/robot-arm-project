"""URDF 处理工具。

背景：
    ``moveit_resources_panda_description`` 提供的 ``panda.urdf`` 只包含运动学与几何
    （供 MoveIt 规划使用），**没有 ``<inertial>``**。而 sdformat 的 URDF 解析器会
    直接丢弃缺少惯量的 link，导致 Gazebo 里生成出一个"没有连杆和关节"的空模型。

因此本模块在 xacro 展开之后，为缺少惯量的 link 注入 Franka 官方质量/惯量参数
（数值取自 ``franka_description`` 的 ``rbots/fr3/inertials.yaml`` 与
``end_effectors/franka_hand/inertials.yaml``，FR3 与 Panda 为同一机械结构的迭代型号），
从而得到既保留官方运动学、又可用于 Gazebo 物理仿真的完整描述。
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, Optional

# link -> (mass, com(x,y,z), inertia(ixx, ixy, ixz, iyy, iyz, izz))
INERTIAL_TABLE: Dict[str, tuple] = {
    'panda_link0': (2.3966, (-0.0172, 0.0004, 0.0745),
                    (0.0090, 0.0, 0.0020, 0.0115, 0.0, 0.0085)),
    'panda_link1': (2.9274653454,
                    (0.0000004128, -0.0181251324, -0.0386035970),
                    (0.023927316485107913, 1.3317903455714081e-05,
                     -0.00011404774918616684, 0.0224821613275756,
                     -0.0019950320628240115, 0.006350098258530016)),
    'panda_link2': (2.9355370338,
                    (0.0031828864, -0.0743221644, 0.0088146084),
                    (0.041938946257609425, 0.00020257331521090626,
                     0.004077784227179924, 0.02514514885014724,
                     -0.0042252158006570156, 0.06170214472888839)),
    'panda_link3': (2.2449013699,
                    (0.0407015686, -0.0048200565, -0.0289730823),
                    (0.02410142547240885, 0.002404694559042109,
                     -0.002856269270114313, 0.01974053266708178,
                     -0.002104212683891874, 0.019044494482244823)),
    'panda_link4': (2.6155955791,
                    (-0.0459100965, 0.0630492960, -0.0085187868),
                    (0.03452998321913202, 0.01322552265982813,
                     0.01015142998484113, 0.028881621933049058,
                     -0.0009762833870704552, 0.04125471171146641)),
    'panda_link5': (2.3271207594,
                    (-0.0016039605, 0.0292536262, -0.0972965990),
                    (0.051610278463662895, -0.005715173387783472,
                     -0.0035673167625872135, 0.04787729713371481,
                     0.010673985108535986, 0.016423625579357254)),
    'panda_link6': (1.8170376524,
                    (0.0597131221, -0.0410294666, -0.0101692726),
                    (0.005412333594383447, 0.006193456360285834,
                     0.0014219289306117652, 0.014058329545509979,
                     -0.0013140753741120031, 0.016080817924212554)),
    'panda_link7': (0.6271432862,
                    (0.0045225817, 0.0086261921, -0.0161633251),
                    (0.00021092389150104718, -2.433299114461931e-05,
                     4.564480393778983e-05, 0.00017718568002411474,
                     8.744070223226438e-05, 5.993190599659971e-05)),
    # panda_link8 在官方描述中是纯坐标帧（无几何/无惯量），给一个极小但正定的惯量
    'panda_link8': (0.1000, (0.0, 0.0, 0.0), (2.0e-4, 0.0, 0.0, 2.0e-4, 0.0, 2.0e-4)),
    'panda_hand': (0.6544, (-0.0000376, 0.0119128, 0.0207260),
                   (0.00186, 0.0, 0.0, 0.00030, -0.00002, 0.00174)),
    'panda_leftfinger': (0.0291, (0.0, 0.0152850, 0.0219675),
                         (0.00000849, 0.0, 0.0, 0.00000853, -0.00000106, 0.00000177)),
    'panda_rightfinger': (0.0291, (0.0, 0.0152850, 0.0219675),
                          (0.00000849, 0.0, 0.0, 0.00000853, -0.00000106, 0.00000177)),
}

DEFAULT_INERTIAL = (0.5, (0.0, 0.0, 0.0), (1.0e-3, 0.0, 0.0, 1.0e-3, 0.0, 1.0e-3))

# 注意：机械臂底座通过名为 world 的 link + 固定关节挂到世界坐标系。
# 该 link 不在此表中、也不注入惯量（Gazebo 视其为世界坐标系，无需惯量；
# 注入反而会让 robot_state_publisher 的 KDL 告警）。详见 inject_inertials。


def _build_inertial(parent: ET.Element, spec: tuple) -> ET.Element:
    mass, com, (ixx, ixy, ixz, iyy, iyz, izz) = spec
    inertial = ET.Element('inertial')

    origin = ET.SubElement(inertial, 'origin')
    origin.set('xyz', f'{com[0]} {com[1]} {com[2]}')
    origin.set('rpy', '0 0 0')

    mass_element = ET.SubElement(inertial, 'mass')
    mass_element.set('value', repr(float(mass)))

    inertia = ET.SubElement(inertial, 'inertia')
    inertia.set('ixx', repr(float(ixx)))
    inertia.set('ixy', repr(float(ixy)))
    inertia.set('ixz', repr(float(ixz)))
    inertia.set('iyy', repr(float(iyy)))
    inertia.set('iyz', repr(float(iyz)))
    inertia.set('izz', repr(float(izz)))
    return inertial


def inject_inertials(urdf_xml: str) -> str:
    """为缺失 ``<inertial>`` 的 link 注入官方惯量参数。

    例外：名为 ``world`` 的 link **不注入**。实测（ign sdf -p）该 link 无论有无惯量，
    转出的 SDF 逐字节一致——Gazebo 会把 ``world`` 解释为世界坐标系而非刚体。反之，
    若给它注入惯量，robot_state_publisher 的 KDL 解析器会告警：

        [kdl_parser]: The root link world has an inertia specified in the URDF,
        but KDL does not support a root link with an inertia.

    真实连杆则必须注入：sdformat 会丢弃缺少 ``<inertial>`` 的 link 及其关节
    （实测移除 panda_link1 的惯量后，panda_link1 与 panda_joint1 都会被整体移除）。
    """
    root = ET.fromstring(urdf_xml)
    for link in root.findall('link'):
        if link.find('inertial') is not None:
            continue
        name = link.get('name', '')
        if name == 'world':
            continue
        spec = INERTIAL_TABLE.get(name, DEFAULT_INERTIAL)
        inertial = _build_inertial(link, spec)
        # URDF 规范要求 inertial 位于 visual/collision 之前
        link.insert(0, inertial)

    return ET.tostring(root, encoding='unicode')


# 夹爪手指的碰撞体用简单盒体替代原始 STL。
# 依据官方 finger.stl 实测包围盒：x∈[-0.0105,0.0105]，y∈[0,0.0264]，z∈[0,0.0538]。
# 使用原语碰撞体可以：1) 规避部分环境下 package:// 网格解析失败导致"无碰撞体"的隐患；
# 2) 让 ODE 接触计算更稳定（网格-网格接触容易出现穿透）。
# 【主动变薄】闭合轴(y)方向由真实 26.5mm 缩到 10mm：真实 Franka 手指沿闭合轴有
# 26.5mm 厚度，在紧邻布局下手指本体必然插进邻居、导致下压不可达。仿真里 MoveIt 规划
# 场景与 Gazebo 物理用同一份 URDF 生成的同一碰撞盒，故改薄后两边一致、确实能塞进更小的
# 缝；代价是偏离真实夹爪几何，真机迁移会失效（与 system.yaml 的 layout.gap 配套）。
# origin 的 y 取盒厚一半，保证手指**接触面仍落在关节平面 y=0**（不改变夹持基准）。
FINGER_BOX = (0.021, 0.010, 0.0537)
# 左手指导轨沿 +y 偏移，右手指导轨在原描述中绕 z 旋转 180°（即镜像到 -y）
FINGER_BOX_ORIGIN = {
    'panda_leftfinger': (0.0, 0.005, 0.0269),
    'panda_rightfinger': (0.0, -0.005, 0.0269),
}
FINGER_LINKS = ('panda_leftfinger', 'panda_rightfinger')


def simplify_finger_collision(urdf_xml: str) -> str:
    """把手指导轨的碰撞几何替换为盒体原语（镜像关系与原始网格一致）。"""
    root = ET.fromstring(urdf_xml)
    for link in root.findall('link'):
        name = link.get('name')
        if name not in FINGER_LINKS:
            continue
        collisions = link.findall('collision')
        if not collisions:
            continue
        link.remove(collisions[0])
        collision = ET.Element('collision')
        collision.set('name', 'finger_collision')
        origin = ET.SubElement(collision, 'origin')
        origin.set('xyz', '{} {} {}'.format(*FINGER_BOX_ORIGIN[name]))
        origin.set('rpy', '0 0 0')
        geometry = ET.SubElement(collision, 'geometry')
        # 注意：URDF 的 box 用 size 属性（<box size="x y z"/>），
        # 而 SDF 用子元素（<box><size>x y z</size></box>），写错会导致
        # "Box shape has no size attribute" 并让整个 collision 被丢弃。
        box = ET.SubElement(geometry, 'box')
        box.set('size', '{} {} {}'.format(*FINGER_BOX))
        # 追加到末尾，保持 inertial -> visual -> collision 的 URDF 元素顺序
        link.append(collision)

    return ET.tostring(root, encoding='unicode')


def drop_finger_joint_mimic(urdf_xml: str) -> str:
    """删除 panda_finger_joint2 的 <mimic> 标签。

    官方 URDF 用 mimic 表达"右手指导轨跟随左手"；但 Gazebo/SDF 的 mimic 会由
    物理引擎强制驱动 joint2，从而与 ros2_control 对 joint2 的力控指令冲突
    （表现为夹持力不足、物体在搬运中滑落）。本项目由 JointGroupEffortController
    同时下发力矩，因此不需要引擎侧的 mimic。
    """
    root = ET.fromstring(urdf_xml)
    for joint in root.findall('joint'):
        if joint.get('name') in ('panda_finger_joint1', 'panda_finger_joint2'):
            for mimic in joint.findall('mimic'):
                joint.remove(mimic)
    return ET.tostring(root, encoding='unicode')


def raise_finger_effort_limit(urdf_xml: str, limit: float = 40.0) -> str:
    """把手指关节的 effort 上限从官方 20N 提高（默认 40N）。

    Gazebo Sim / gz_ros2_control 会按 URDF limit 截断关节力指令。本项目的夹持
    策略是"闭合轻触（close 16N，防止撞偏物体）、保持加力（hold 30N，抵抗抬升
    加速度造成的楔开/滑脱）"：若上限仍为 20N，保持力会被截断而达不到设计值。
    """
    root = ET.fromstring(urdf_xml)
    for joint in root.findall('joint'):
        if joint.get('name') in ('panda_finger_joint1', 'panda_finger_joint2'):
            limit_element = joint.find('limit')
            if limit_element is not None:
                limit_element.set('effort', repr(float(limit)))
    return ET.tostring(root, encoding='unicode')


def load_robot_description(xacro_path: str, mappings: Optional[dict] = None) -> str:
    """xacro 展开 + 惯量注入 + 手指碰撞体简化，返回可用于 Gazebo / MoveIt 的 URDF。"""
    import xacro

    document = xacro.process_file(xacro_path, mappings=mappings or {})
    urdf = inject_inertials(document.toxml())
    urdf = simplify_finger_collision(urdf)
    urdf = drop_finger_joint_mimic(urdf)
    return raise_finger_effort_limit(urdf)


def main():
    """命令行入口：把处理后的 URDF 打印到标准输出。"""
    import argparse

    parser = argparse.ArgumentParser(description='输出注入惯量后的 Panda URDF')
    try:
        from ament_index_python.packages import get_package_share_directory
        default_xacro = os.path.join(
            get_package_share_directory('sorting_simulation'),
            'urdf', 'panda_gz.urdf.xacro',
        )
    except Exception:
        default_xacro = ''
    parser.add_argument('--xacro', default=default_xacro, help='xacro 文件路径')
    parser.add_argument('--output', default='', help='输出文件（默认打印到 stdout）')
    args = parser.parse_args()

    urdf = load_robot_description(os.path.abspath(args.xacro))
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as handle:
            handle.write(urdf)
    else:
        print(urdf)


if __name__ == '__main__':
    main()