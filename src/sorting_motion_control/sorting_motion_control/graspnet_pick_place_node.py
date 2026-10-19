"""模块5/P2：GraspNet 6DoF 抓取执行节点（方案A 链路末端）。

链路（方案A，已砍掉 2D 检测/垂直抓取旧链路）：
    Gazebo 点云 -> pointcloud_preprocessor -> graspnet_planner_node
        -> /sorting/graspnet_grasps -> 本节点 -> MoveIt 执行抓取-放置

本节点职责：
    1. 订阅 GraspNet 输出的 6DoF 抓取候选（world 系）
    2. 把每条抓取映射为 panda_link8 的末端位姿（见下方坐标系约定）
    3. 按评分排序，逐条尝试 MoveIt 规划+执行；全部失败才判本次抓取失败
    4. 抓取失败/成功均发布 statistics 与 task_status，供评估节点记录

坐标系约定（重要）：
    * GraspNet 输入点云为相机光学系，输出抓取已由 graspnet_planner_node 转到 world 系。
    * 仿真中机器人由 ros_gz_sim 以 (0,0,0)、无旋转生成（见 sim_bringup_gz.launch.py），
      因此 panda_link0 与 world 完全重合，world 系抓取可直接作为 MoveIt 的位姿目标，
      无需再做一次 base<-world 变换。

GraspNet 姿态约定（取自第三方源码，非猜测）：
    graspnet-baseline/utils/loss_utils.py: batch_viewpoint_params_to_matrix
        R 的三列为 [axis_x, axis_y, axis_z]，其中 axis_x 即 approach 方向；
    graspnetAPI/utils/utils.py: plot_gripper_pro_max
        两指沿 R 的第 2 列（index=1）分开 -> 该列为两指闭合方向。
    归纳：R[:,0]=approach（指向物体内部）、R[:,1]=closing（两指闭合方向）、
         R[:,2]=height。

Panda 末端映射（沿用本项目旧垂直抓取链路已验证的结论）：
    panda_link8 的 +z 轴指向指尖，即抓取接近方向，故 R_link8_z = approach；
    两指闭合方向在 link8 坐标系中恒为 normalize(1,1,0)（由 panda_hand 相对
    panda_link8 绕 z 的固定安装角决定），故约束
        R_link8 @ normalize(1,1,0) = closing（投影到垂直于 approach 的平面）。
    该构造在 approach=(0,0,-1) 时退化为旧链路使用的“垂直下抓 + yaw”姿态，
    已验证一致（可用它做单元回归）。
"""

import copy
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from sorting_bringup import load_system_config
from sorting_interfaces.msg import Grasp6DArray, ObjectStateArray, SortingStatistics
from sorting_interfaces.srv import StartSorting

from .gripper import GripperController
from .moveit_client import MoveGroupClient

# 两指闭合方向在 panda_link8 坐标系中的方向（panda_hand 相对 link8 绕 z 的安装角所致）
CLOSING_AXIS_LINK8 = np.array([math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0])

# panda_link8 -> 指尖（裸指末端）沿 approach 的距离（米）。由 URDF 精确推导，非估计值：
#   panda_hand_joint:    link8 -> hand，平移 0（仅绕 z 转 -45°）
#   panda_finger_joint1: hand -> 手指关节原点，z = +0.0584
#   urdf_utils.FINGER_BOX 高 0.0537，指尖为手指盒远端 z = +0.0537
# 合计 0.0584 + 0.0537 = 0.1121 m。
# 用途：把 GraspNet 的"抓取中心"换算成真实指尖落点，做桌面干涉诊断
# （旧配置注释按 0.105 估算，比真值短 7mm，是"下压不可达/空抓"的重要嫌疑）。
DEFAULT_TCP_LENGTH = 0.1121

# ---------------------------------------------------------------------------
# 手指碰撞盒几何（与 sorting_simulation.urdf_utils.simplify_finger_collision 替换后
# 的真实碰撞体一致，勿随意改）：
#   FINGER_BOX       = (0.021, 0.010, 0.0537)   —— 碰撞盒尺寸（x, y, z），y 已主动变薄
#   FINGER_BOX_ORIGIN                            —— 盒中心相对手指关节原点的偏移
#   FINGER_JOINT_Z   = 0.0584                    —— panda_finger_joint 的 origin z
#   HAND_Z_ROT       = -pi/4                     —— panda_hand 相对 panda_link8 的安装角
# 为什么需要它：判断"下压会不会碰到桌面"要看手指**碰撞盒的 8 个角点**里最低的那个，
# 而不是沿 approach 的指尖点——夹爪倾斜时盒角会比指尖点更低，实测缺口可达 ~23mm。
# ---------------------------------------------------------------------------
FINGER_BOX_SIZE = (0.021, 0.010, 0.0537)
FINGER_BOX_ORIGIN = {
    'left': (0.0, 0.005, 0.0269),
    'right': (0.0, -0.005, 0.0269),
}
FINGER_JOINT_Z = 0.0584
HAND_Z_ROT = -math.pi / 4.0


def finger_box_corners_link8(finger_open: float) -> np.ndarray:
    """两指碰撞盒 8 角点在 panda_link8 系中的坐标，返回 (16, 3)。

    手指沿 hand 的 ±y 张开 ``finger_open``（米），因此张开量会平移盒角；夹爪倾斜时
    该偏移带来额外下垂。此处精确建模，供"逐候选桌面间隙"计算使用。
    """
    half = np.array(FINGER_BOX_SIZE, dtype=float) / 2.0
    c, s = math.cos(HAND_Z_ROT), math.sin(HAND_Z_ROT)
    r_hand = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    corners = []
    for side, sign in (('left', 1.0), ('right', -1.0)):
        origin = np.array(FINGER_BOX_ORIGIN[side], dtype=float)
        joint = np.array([0.0, sign * finger_open, FINGER_JOINT_Z])
        for dx in (-half[0], half[0]):
            for dy in (-half[1], half[1]):
                for dz in (-half[2], half[2]):
                    p_finger = origin + np.array([dx, dy, dz])
                    corners.append(r_hand @ (p_finger + joint))
    return np.array(corners)


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(m: np.ndarray):
    """3x3 旋转矩阵 -> (x, y, z, w) 四元数。"""
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s)


def _matrix_to_pose(m: np.ndarray) -> Pose:
    """4x4 齐次变换 -> geometry_msgs/Pose。"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in m[:3, 3])
    qx, qy, qz, qw = _matrix_to_quat(m[:3, :3])
    pose.orientation.x, pose.orientation.y = float(qx), float(qy)
    pose.orientation.z, pose.orientation.w = float(qz), float(qw)
    return pose


def link8_orientation(approach, closing) -> np.ndarray:
    """由 GraspNet 的 approach / closing 方向构造 panda_link8 的姿态矩阵。

    约束：
        R[:,2] = approach                      （link8 +z = 指尖方向 = 接近方向）
        R @ normalize(1,1,0) = closing 的垂直分量（保持两指沿 grasp 闭合方向）
    """
    a = np.asarray(approach, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-9)

    c = np.asarray(closing, dtype=float)
    u = c - float(np.dot(c, a)) * a
    if np.linalg.norm(u) < 1e-6:
        # 退化：closing 与 approach 近乎共线，任取一个垂直方向
        ref = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        u = np.cross(a, ref)
    u = u / np.linalg.norm(u)

    # 构造垂直于 approach 的正交基 (p, r)，使 (p, r, a) 右手系
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    p = np.cross(a, ref)
    p = p / np.linalg.norm(p)
    r = np.cross(a, p)

    # 闭合方向 = cos(φ+45°)·p + sin(φ+45°)·r，求 φ 使之为 u
    theta = math.atan2(float(np.dot(u, r)), float(np.dot(u, p)))
    phi = theta - math.pi / 4.0
    axis_x = math.cos(phi) * p + math.sin(phi) * r
    axis_y = -math.sin(phi) * p + math.cos(phi) * r
    return np.column_stack([axis_x, axis_y, a])


def _vertical_pose(x: float, y: float, z: float) -> Pose:
    """垂直向下、两指沿 world +X 闭合的末端位姿（投放/观察等固定姿态用）。"""
    r_matrix = link8_orientation([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    qx, qy, qz, qw = _matrix_to_quat(r_matrix)
    pose.orientation.x, pose.orientation.y = float(qx), float(qy)
    pose.orientation.z, pose.orientation.w = float(qz), float(qw)
    return pose


def _fmt(value) -> str:
    """数值格式化（None 显示为 —），用于抓取验证日志。"""
    return '—' if value is None else f'{value:.4f}'


def _fmt_frac(value) -> str:
    """比例格式化（None 显示为 —）。"""
    return '—' if value is None else f'{value * 100:.0f}%'


def _ik_text(result) -> str:
    """把 compute_ik 的结果格式化为一行可读文本（None 表示服务不可用）。"""
    if result is None:
        return '服务不可用'
    if result['error'] == 1:
        values = ', '.join(f'{v:+.3f}' for v in result['positions'])
        return f'成功 [{values}]'
    return result['error_name']


class GraspNetPickPlaceNode(Node):
    """GraspNet 6DoF 抓取执行。"""

    def __init__(self):
        super().__init__('graspnet_pick_place_node')

        cfg = load_system_config()
        frames = cfg['frames']
        topics = cfg['topics']
        motion_cfg = cfg['motion']
        gripper_cfg = cfg['gripper']
        grasp_cfg = cfg['grasp']
        scene_cfg = cfg['scene']
        gcfg = cfg.get('graspnet', {})

        self.group = ReentrantCallbackGroup()

        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7',
        ]
        self.moveit = MoveGroupClient(
            self,
            group_name='panda_arm',
            ee_link=frames['ee_link'],
            base_frame=frames['base'],
            joint_names=self.joint_names,
            planner_id=motion_cfg.get('planner_id', 'RRTConnect'),
            position_tolerance=float(motion_cfg.get('goal_position_tolerance', 0.006)),
            orientation_tolerance=float(motion_cfg.get('goal_orientation_tolerance', 0.08)),
            planning_time=float(motion_cfg.get('planning_time', 5.0)),
            planning_attempts=int(motion_cfg.get('planning_attempts', 5)),
        )
        self.gripper = GripperController(self, gripper_cfg)

        self.velocity = float(motion_cfg.get('velocity_scaling', 0.25))
        self.acceleration = float(motion_cfg.get('accel_scaling', 0.25))
        self.approach_velocity = float(motion_cfg.get('approach_velocity_scaling', 0.10))
        self.cartesian_velocity = float(motion_cfg.get('cartesian_velocity_scaling', 0.15))
        self.home_joint_values = list(motion_cfg.get('home_joint_values', []))
        # 观察位：DETECT 期间把机械臂停到这里，避免它出现在相机视野里（见 _park_for_detection）
        self.observe_joint_values = (list(motion_cfg.get('observe_joint_values', []))
                                     or self.home_joint_values)

        # ---- GraspNet 执行参数 ----
        # tcp_offset：panda_link8 到"抓取中心"的距离（沿 approach 方向）。GraspNet 的
        # translation 为两指之间的抓取中心，因此 link8 需沿 -approach 后退该距离。
        self.tcp_offset = float(gcfg.get('tcp_offset', 0.105))
        # link8 -> 指尖沿 approach 的距离（米）。用于诊断把"抓取中心"换算为指尖落点。
        self.tcp_length = float(gcfg.get('tcp_length', DEFAULT_TCP_LENGTH))
        # ---- 逐候选 tcp_offset 补偿（Step1）----
        # 下压前手指保持张开，故用张开位计算碰撞盒角点；余量为角点与桌面的最小允许间隙。
        self.tcp_offset_max = float(gcfg.get('tcp_offset_max', 0.16))
        self.table_clearance_margin = float(gcfg.get('table_clearance_margin', 0.005))
        open_position = float(gripper_cfg.get('open_position', 0.038))
        self._finger_corners_link8 = finger_box_corners_link8(open_position)
        self.pregrasp_retreat = float(gcfg.get('pregrasp_retreat', 0.12))
        self.lift_retreat = float(gcfg.get('lift_retreat', 0.15))
        # 单物体尝试上限：同一物体最多尝试 N 个最高分候选，全失败即入黑名单
        self.max_attempts_per_object = int(gcfg.get('max_attempts_per_object', 2))
        # 候选预筛查数量：对每组前 N 个候选做"无碰撞逆解"预检，可行候选排到前面优先尝试。
        # 物体靠得近时，最高分候选的下压路径常被邻物挡住；不预筛时只试前 2 个高分候选
        # 会全部失败并误判物体不可抓（见 _screen_candidates）。
        self.candidate_screen_topk = int(gcfg.get('candidate_screen_topk', 8))
        # 动态 ACM：规划前只把"夹爪连杆 <-> 当前目标物体"设为允许碰撞，其余
        # （夹爪<->其他物体/桌面、手臂<->一切）一律保持严格；抓取结束（成功或失败）
        # 后立即恢复基线。这样下压段可以放心地用 avoid_collisions=True，不再需要
        # 整体关闭碰撞检测。列表为实际会与目标接触的夹爪连杆。
        self.acm_gripper_links = list(gcfg.get('acm_gripper_links', [
            'panda_leftfinger', 'panda_rightfinger',
            'panda_leftfinger_pad', 'panda_rightfinger_pad',
        ]))
        # 把无物体 ID 的抓取候选按空间邻近聚成"物体组"的半径（米）
        self.object_cluster_radius = float(gcfg.get('object_cluster_radius', 0.03))
        # 连续模式下，单轮拿不到抓取候选时不立即收工：冷启动/瞬时遮挡都可能出现空帧，
        # 连续空轮达到该次数才判定"确实无可抓目标"。0/1 表示首轮为空即结束。
        self.max_empty_rounds = int(gcfg.get('max_empty_rounds', 3))
        # ---- 可抓包络（严格）----
        # 只有物体中心落在该矩形内才视为"可抓"：包络外的物体一律从规划场景移除、不参与抓取。
        # 该包络必须严格小于机械臂真实可达范围（留安全余量），确保包络内物体都能正常夹取，
        # 从而杜绝"机械臂卡住待机、场上仍有可抓物体"。可用 scripts/probe_grasp_envelope.py 实测收紧。
        env = scene_cfg.get('grasp_envelope') or {}
        self.grasp_envelope = (
            float(env.get('x_min', 0.18)), float(env.get('x_max', 0.58)),
            float(env.get('y_min', -0.34)), float(env.get('y_max', 0.34)),
        )
        # 本轮黑名单（物体组中心）：防"抓取鬼打墙"——同一物体反复失败后直接放弃
        self._blacklist = []
        drop = gcfg.get('drop_pose', [0.66, 0.0, 0.22])
        self.drop_pose = _vertical_pose(float(drop[0]), float(drop[1]), float(drop[2]))
        # 默认投放点仅作为"无类别信息"时的兜底；有类别时按分拣区投放（见 _drop_pose_for）
        self.default_drop_pose = copy.deepcopy(self.drop_pose)

        # ---- 场景碰撞体 ----
        self.table_top_z = float(scene_cfg['table']['top_z'])
        self.table_size = list(scene_cfg['table']['size'])
        self.table_pose = list(scene_cfg['table']['pose'])
        self.bin_base_thickness = float(grasp_cfg.get('bin_base_thickness', 0.008))
        self.bins = list(scene_cfg.get('bins', []))
        self.class_table = None
        try:
            from sorting_bringup import ClassTable
            self.class_table = ClassTable()
        except Exception:
            self.class_table = None

        # ---- GraspNet 抓取缓存 ----
        self._latest_grasps = None
        self._grasp_seq = 0
        self.create_subscription(
            Grasp6DArray, gcfg.get('output_grasps', '/sorting/graspnet_grasps'),
            self._on_grasps, 5, callback_group=self.group,
        )

        # ---- 聚类质心/尺寸缓存（诊断用）----
        # 预处理节点发布的 CUBE marker：位姿为簇质心、scale 为簇包围盒尺寸。
        # 用它在候选执行前判断"抓取中心是否横向偏到物体之外"。
        self._clusters = []
        self.create_subscription(
            MarkerArray,
            cfg.get('perception', {}).get('output_markers', '/sorting/clusters'),
            self._on_clusters, 5, callback_group=self.group,
        )

        # ---- 待抓物体 -> 规划场景碰撞体 ----
        # 规划场景原先只有桌面与分拣箱、**不含待抓物体**，于是 MoveIt 认为下压
        # 100% 无碰撞，而 Gazebo 里手指实际压在物体顶面上 -> 手臂被顶住 ->
        # panda_arm_controller 报 PATH_TOLERANCE_VIOLATED（MoveIt 错误码 -4）。
        # 这里按真值位姿 + 标称尺寸把物体补进场景，使"指尖落到物体上"这类坏候选
        # 在**规划层**就被显式拒绝，而不是留到物理层卡死（难以定位）。
        # 说明：位姿取仿真真值，尺寸即 classes.yaml 标称值，与项目既有的
        # "已知 SKU 尺寸"假设一致；真实部署时该位姿应由位姿估计提供。
        self._gt_objects = {}
        self._gt_zones = {}
        self._scene_object_ids = set()
        self._attached_ids = set()
        # 上次写入规划场景的物体位姿：用于判断物体是否移动；移动后必须重发 ADD，
        # 否则规划场景/rviz 会一直停在旧位置，规划器对着幽灵物体规划导致卡死
        self._scene_object_poses = {}
        # 世界系物体盒的安全余量（每边）：把"指面到物体的余量"不足的贴边候选
        # 在规划层拒掉，避免留到物理层变成硬接触（MoveIt -4）
        self.scene_object_margin = float(gcfg.get('scene_object_margin', 0.0))
        self.scene_client = None
        self.create_subscription(
            ObjectStateArray, topics.get('ground_truth', '/sorting/ground_truth'),
            self._on_ground_truth, 5, callback_group=self.group,
        )

        # ---- 手臂关节最新位置（attach 时用 FK 把物体位姿换算到 panda_hand 系）----
        self._arm_positions = None
        self.create_subscription(
            JointState, '/joint_states', self._on_arm_state, 10, callback_group=self.group,
        )

        # ---- 评估输出 ----
        self.pub_statistics = self.create_publisher(SortingStatistics, topics['statistics'], 10)
        self.pub_status = self.create_publisher(String, topics['task_status'], 10)
        # 供 rviz 复核 tcp_offset：显示"抓取中心"与"实际下发的 link8 位姿"
        self.pub_markers = self.create_publisher(MarkerArray, '/sorting/execution_markers', 3)

        self.stats = {
            'attempted': 0, 'picked_success': 0, 'grasp_failures': 0,
            'retries': 0, 'plan_failures': 0,
            # 分项计数（供评估层量化空抓率/放置成功率）
            'empty_grasps': 0,      # 下压到位但闭合到底未夹住（空抓）
            'grasp_confirmed': 0,   # 闭合确认夹住（放置阶段的尝试次数）
            'lift_slip': 0,         # 已夹住但抬升失败（搬运/放置途中掉落）
            'drop_plan_fail': 0,    # 已夹住但投放点上方规划失败
        }
        # 规划请求计数基线：EVAL 只统计本轮抓取任务期间发出的规划请求
        self._plan_baseline = {'plan_requests': 0, 'plan_successes': 0}
        # 本轮内是否疑似把目标物体碰移（夹空/掉落）——置位后放弃本轮剩余候选、重新感知
        self._object_disturbed = False
        self._cycle_times = []
        self._running = False
        self._state = 'IDLE'
        self.create_timer(0.5, self._publish_statistics, callback_group=self.group)

        self.create_service(
            StartSorting, cfg['services']['start_sorting'], self.on_start_sorting,
            callback_group=self.group,
        )

        self._bootstrapped = False
        self._bootstrapping = False
        self._scene_ready = False
        self.create_timer(2.0, self.bootstrap, callback_group=self.group)

        self.get_logger().info('GraspNet 抓取执行节点启动（方案A链路末端）')

    # ==================================================================
    # 启动引导
    # ==================================================================
    def bootstrap(self):
        if self._bootstrapped or self._bootstrapping:
            return
        self._bootstrapping = True
        try:
            if not self.moveit.wait_for_server(timeout=5.0):
                self.get_logger().warn('等待 /move_action 服务中...')
                return
            if not self._scene_ready:
                self._setup_planning_scene()
                self._scene_ready = True
            if self.home_joint_values:
                ok, message = self.moveit.move_to_joint_values(
                    self.home_joint_values, self.velocity, self.acceleration, timeout=30.0
                )
                self.get_logger().info(f'回到初始位姿: {ok} ({message})')
                if not ok:
                    # 冷启动时控制器（gz_ros2_control）可能尚未就绪，MoveIt 会返回 99999。
                    # 此时**不得**宣告就绪，否则 start_sorting 会被误接受、随即空转退出；
                    # 保持未就绪，交给 2s 定时器下一轮重试。
                    self.get_logger().warn('初始位姿规划失败（控制器可能未就绪），稍后重试')
                    return
            self.gripper.open()
            self._bootstrapped = True
            self.get_logger().info('GraspNet 执行节点就绪')
        finally:
            self._bootstrapping = False

    def _setup_planning_scene(self):
        """把桌面与分拣箱作为碰撞体加入规划场景。"""
        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'work_table'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(v) for v in self.table_size]
        table.primitives.append(box)
        table_pose = Pose()
        table_pose.position.x = float(self.table_pose[0])
        table_pose.position.y = float(self.table_pose[1])
        table_pose.position.z = float(self.table_pose[2])
        table_pose.orientation.w = 1.0
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table)

        bin_size = 0.16
        bin_height = self.bin_base_thickness + 0.03
        for zone_name in self.bins:
            if self.class_table is not None:
                zone_pose = self.class_table.zone_pose(zone_name)
            else:
                continue
            obstacle = CollisionObject()
            obstacle.header.frame_id = 'world'
            obstacle.id = f'bin_{zone_name}'
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [bin_size, bin_size, bin_height]
            obstacle.primitives.append(primitive)
            pose = Pose()
            pose.position.x = float(zone_pose[0])
            pose.position.y = float(zone_pose[1])
            pose.position.z = bin_height / 2.0
            pose.orientation.w = 1.0
            obstacle.primitive_poses.append(pose)
            obstacle.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obstacle)

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/apply_planning_scene 不可用，跳过碰撞环境配置')
            return
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if future.done() and future.result() is not None:
            self.get_logger().info(f'规划场景已加入桌面与 {len(scene.world.collision_objects) - 1} 个分拣箱')
        else:
            self.get_logger().warn('规划场景配置失败')

    # ==================================================================
    # 抓取缓存 / 状态发布
    # ==================================================================
    def _on_grasps(self, msg: Grasp6DArray):
        self._latest_grasps = msg
        self._grasp_seq += 1

    def _on_ground_truth(self, msg: ObjectStateArray):
        """缓存真值物体：名字 -> (位姿, 尺寸, 是否已被抓取)。"""
        self._gt_objects = {
            obj.object_name: (obj.pose, obj.size, bool(obj.is_grasped))
            for obj in msg.objects
        }
        # 物体 -> 应投放的分拣区（来自真值 placement_zone），供 _drop_pose_for 使用
        self._gt_zones = {obj.object_name: obj.placement_zone for obj in msg.objects}

    def _on_arm_state(self, msg: JointState):
        """缓存七个手臂关节位置（attach 做 FK 用）。"""
        position = dict(zip(msg.name, msg.position))
        if all(name in position for name in self.joint_names):
            self._arm_positions = [float(position[name]) for name in self.joint_names]

    def _apply_scene(self, scene, timeout: float = 5.0, attempts: int = 3) -> bool:
        """把 PlanningScene 差异发给 /apply_planning_scene；带服务等待与重试。

        为何要重试：本项目实测过 DDS/服务偶发不响应（单次调用超时）。一旦发生在
        "附着/撤销附着"上，就会造成节点与 MoveIt 状态不一致——夹爪上留一个幽灵物体，
        之后所有运动被判碰撞/轨迹无效（-2），机械臂卡死、挡住相机、点云变空。
        """
        if self.scene_client is None:
            return False
        for attempt in range(max(1, attempts)):
            if not self.scene_client.wait_for_service(timeout_sec=2.0):
                time.sleep(0.2)
                continue
            future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.01)
            if future.done() and future.result() is not None:
                return True
            self.get_logger().warn(
                f'[scene] /apply_planning_scene 第 {attempt + 1}/{attempts} 次未响应，重试')
            time.sleep(0.2)
        return False

    def _nearest_gt_object(self, center) -> str:
        """离抓取中心最近的真值物体名（即当前被夹住的那一个）。"""
        best = None
        for name, (pose, _size, _grasped) in self._gt_objects.items():
            point = np.array([pose.position.x, pose.position.y, pose.position.z])
            distance = float(np.linalg.norm(np.asarray(center, dtype=float) - point))
            if best is None or distance < best[1]:
                best = (name, distance)
        return None if best is None else best[0]

    def _drop_pose_for(self, object_name) -> Pose:
        """按物体所属分拣区返回投放位姿；无类别信息时退回默认投放点。

        分拣区位置取自 classes.yaml 的 zones（zone_pose 的 z 为箱底/桌面高度），
        投放高度沿用默认 drop_pose 的 z，保证与既有下放动作一致。
        """
        zone = self._gt_zones.get(object_name) if object_name else None
        if zone and self.class_table is not None:
            x, y, _z = self.class_table.zone_pose(zone)
            return _vertical_pose(float(x), float(y), float(self.default_drop_pose.position.z))
        return self.default_drop_pose

    def _attach_object(self, object_name: str, link_name: str = 'panda_hand') -> bool:
        """抓取确认后，把物体从世界系摘下并挂到夹爪上（MoveIt 标准做法）。

        不这么做的话，规划器会把"夹爪手里的物体"与"规划场景里同一物体的副本"
        判为碰撞，抬升/搬运轨迹全部被拒（实测 MoveIt -2 INVALID_MOTION_PLAN）。
        附着后物体随 ``link_name`` 一起运动，手指/指尖垫/手掌列入 touch_links
        豁免接触。

        物体位姿换算：AttachCollisionObject 要求形状位姿给在**所附着连杆的系**下，
        因此用 FK 取连杆世界位姿，再算 T_link_obj = T_world_link^-1 * T_world_obj。
        """
        entry = self._gt_objects.get(object_name)
        if entry is None:
            self.get_logger().warn(f'attach 跳过：真值中没有 {object_name}')
            return False
        if self._arm_positions is None:
            self.get_logger().warn('attach 跳过：尚未收到 /joint_states')
            return False

        obj_pose, size, _grasped = entry
        hand = self.moveit.compute_fk(self.joint_names, self._arm_positions, link_name)
        if hand is None:
            self.get_logger().warn(f'attach 跳过：{link_name} 的 FK 不可用')
            return False

        t_link = np.eye(4)
        t_link[:3, :3] = _quat_to_matrix(hand[3], hand[4], hand[5], hand[6])
        t_link[:3, 3] = hand[:3]
        t_obj = np.eye(4)
        t_obj[:3, :3] = _quat_to_matrix(obj_pose.orientation.x, obj_pose.orientation.y,
                                        obj_pose.orientation.z, obj_pose.orientation.w)
        t_obj[:3, 3] = [obj_pose.position.x, obj_pose.position.y, obj_pose.position.z]
        t_rel = np.linalg.inv(t_link) @ t_obj

        obj_id = f'pick_{object_name}'
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object.header.frame_id = link_name
        attached.object.id = obj_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size.x), float(size.y), float(size.z)]
        attached.object.primitives.append(primitive)
        attached.object.primitive_poses.append(_matrix_to_pose(t_rel))
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = [
            link_name, 'panda_leftfinger', 'panda_rightfinger',
            'panda_leftfinger_pad', 'panda_rightfinger_pad',
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        # 同步把世界系里的同一物体移出，避免"世界里一份 + 手上又一份"
        world_remove = CollisionObject()
        world_remove.header.frame_id = 'world'
        world_remove.id = obj_id
        world_remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(world_remove)

        if not self._apply_scene(scene):
            self.get_logger().warn(f'attach 失败：{obj_id}')
            return False
        self._scene_object_ids.discard(obj_id)
        self._scene_object_poses.pop(obj_id, None)
        self._attached_ids.add(obj_id)
        self.get_logger().info(f'已把 {obj_id} 附着到 {link_name}（抬升/搬运将不再判为碰撞）')
        return True

    def _clear_all_attached(self):
        """强制清空规划场景里所有"附着在夹爪上的物体"，修复 attach/detach 不一致的幽灵。

        若"撤销附着"因服务/DDS 偶发失败而未生效，MoveIt 会一直把物体挂在夹爪上；物体
        早已落进分拣框，但规划器认为夹爪上还带着它，导致后续所有运动被判碰撞/轨迹无效
        （-2），机械臂卡死、挡住相机、点云变空。这里无条件全部摘掉，世界系副本由下一轮
        _sync_object_collision_objects 按真值放回。
        """
        if not self._attached_ids:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        for obj_id in sorted(self._attached_ids):
            attached = AttachedCollisionObject()
            attached.link_name = 'panda_hand'
            attached.object.header.frame_id = 'panda_hand'
            attached.object.id = obj_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
        ok = self._apply_scene(scene)
        self.get_logger().warn(
            f'[修复] 强制清除幽灵附着物体 {sorted(self._attached_ids)}'
            f'（{"成功" if ok else "失败，下次再试"}）')
        if ok:
            self._attached_ids.clear()

    def _detach_object(self, object_name: str, link_name: str = 'panda_hand') -> bool:
        """释放后把物体从夹爪上摘掉。

        世界系副本由下一轮的 ``_sync_object_collision_objects`` 按新的真值位姿
        重新放回，因此这里只需撤销附着。
        """
        obj_id = f'pick_{object_name}'
        if obj_id not in self._attached_ids:
            return False
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object.header.frame_id = link_name
        attached.object.id = obj_id
        attached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        # 关键：撤销附着的同时，把**世界系同名副本**也移除。MoveIt 在"撤销附着"时会把物体
        # 按当时的夹爪位姿放回世界系——那正好是投放点上方的半空，于是 RViz 里出现"投放后
        # 物体悬空"的假象（其实 Gazebo 里它已落进分拣框）。这里顺手把那份副本清掉，
        # 与 _attach_object 里的 world_remove 对称。
        world_remove = CollisionObject()
        world_remove.header.frame_id = 'world'
        world_remove.id = obj_id
        world_remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(world_remove)
        if not self._apply_scene(scene):
            # 撤销失败时**不能**清 _attached_ids：否则节点以为已摘、MoveIt 仍把它挂在夹爪上，
            # 下一轮 _sync_object_collision_objects 又会放一个世界系副本，形成"手上一个+
            # 世界里一个"的幽灵碰撞体，使后续规划判轨迹无效（MoveIt -2）。
            self.get_logger().warn(f'[detach] 撤销附着失败（{obj_id}），保留附着标记以避免幽灵碰撞体')
            return False
        self._attached_ids.discard(obj_id)
        return True

    def _pose_unchanged(self, obj_id, pose) -> bool:
        """物体位姿相对"上次写入规划场景的位姿"是否基本未变（1e-4 容差）。"""
        prev = self._scene_object_poses.get(obj_id)
        if prev is None:
            return False
        dp = (abs(prev.position.x - pose.position.x)
              + abs(prev.position.y - pose.position.y)
              + abs(prev.position.z - pose.position.z))
        dq = (abs(prev.orientation.x - pose.orientation.x)
              + abs(prev.orientation.y - pose.orientation.y)
              + abs(prev.orientation.z - pose.orientation.z)
              + abs(prev.orientation.w - pose.orientation.w))
        return dp < 1e-4 and dq < 1e-4

    def _sync_object_collision_objects(self):
        """把未抓取的待抓物体按真值同步进规划场景（每轮开始前调用）。

        新增物体发 ADD、消失或已被抓取的物体发 REMOVE；两者都放进同一次
        ApplyPlanningScene 里，避免逐物体往返。物体已在场景中时不做任何调用。
        """
        if not self._gt_objects or self.scene_client is None:
            return

        scene = PlanningScene()
        scene.is_diff = True
        wanted = set()
        for name, (pose, size, grasped) in self._gt_objects.items():
            if grasped:
                continue
            # 可抓包络之外的物体视为"离场"：不进 wanted -> 会被下面的 REMOVE 从规划场景清掉，
            # 避免它卡住机械臂；感知层本就不在其 ROI 内，行为一致。
            if not self._in_grasp_envelope(pose.position.x, pose.position.y):
                continue
            obj_id = f'pick_{name}'
            if obj_id in self._attached_ids:
                # 已挂到夹爪上：世界系不应再有同名副本
                continue
            wanted.add(obj_id)
            # 已在场景中且位姿未变 -> 无需重发；位姿变了（例如被空抓的夹爪顶飞）
            # 必须重发 ADD 把新位姿写进去，否则 rviz/规划器一直停在旧位置
            if obj_id in self._scene_object_ids and self._pose_unchanged(obj_id, pose):
                continue
            obstacle = CollisionObject()
            obstacle.header.frame_id = 'world'
            obstacle.id = obj_id
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            inflate = 2.0 * self.scene_object_margin
            primitive.dimensions = [float(size.x) + inflate,
                                    float(size.y) + inflate,
                                    float(size.z) + inflate]
            obstacle.primitives.append(primitive)
            obstacle.primitive_poses.append(pose)
            obstacle.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obstacle)
            self._scene_object_poses[obj_id] = pose

        # REMOVE 集合不能只看 _scene_object_ids：物体被"附着"时我们从该集合里移除过它，
        # 而事后它可能又以"世界系副本"的形式出现在规划场景里（位姿停在半空）——这种
        # "账本上没有、场景里却有"的悬空盒会永远清不掉，卡死后续所有规划（-2）。
        # 因此按**真值物体全集**来算 REMOVE 集合；当前已附着在夹爪上的除外。
        known_ids = ({f'pick_{name}' for name in self._gt_objects}
                     | set(self._scene_object_ids)) - set(self._attached_ids)
        for obj_id in sorted(known_ids - wanted):
            obstacle = CollisionObject()
            obstacle.header.frame_id = 'world'
            obstacle.id = obj_id
            obstacle.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(obstacle)
            self._scene_object_poses.pop(obj_id, None)
            if obj_id not in self._scene_object_ids:
                self.get_logger().warn(f'[同步] 清除场景中多余的悬空碰撞盒 {obj_id}')

        if not scene.world.collision_objects:
            return

        if self._apply_scene(scene):
            added = wanted - self._scene_object_ids
            removed = self._scene_object_ids - wanted
            self._scene_object_ids = wanted
            if added or removed:
                self.get_logger().info(
                    f'规划场景物体同步：新增 {sorted(added)} 移除 {sorted(removed)}')
        else:
            self.get_logger().warn('规划场景物体同步失败')

    def _acm_set_target(self, target_co_id) -> bool:
        """动态刷新碰撞矩阵（ACM）：只豁免"夹爪连杆 <-> 目标物体"，其余保持严格。

        ``target_co_id`` 为当前目标物体在规划场景里的碰撞体 ID（``pick_<name>``）；
        传 None 表示**恢复基线**（夹爪对所有物体都禁止碰撞）。

        实现（增量修改，务必读全量）：
            1) 读 move_group 当前**完整** ACM；
            2) 把夹爪连杆对**所有**物体的条目先清零（清掉上一轮残留）；
            3) 若给了目标，则把夹爪连杆 <-> 该物体的条目标为允许碰撞；
            4) 连同其余连杆条目一起整体写回（原有自碰撞豁免原样保留）。

        只发少数条目会被 move_group 当作整体替换，抹掉机器人自碰撞豁免、使机械臂
        完全无法运动——这是本项目此前 ACM 尝试失败的根因，因此这里始终读全量再写回。
        """
        current = self.moveit.get_allowed_collision_matrix()
        if current is None:
            self.get_logger().warn('[ACM] 无法读取当前碰撞矩阵，跳过豁免更新')
            return False
        names, values = current

        # 参与更新的物体名 = 矩阵中已有的 pick_* ∪ 当前场景物体 ∪ 已附着物体 ∪ 目标
        object_ids = {name for name in names if name.startswith('pick_')}
        object_ids |= set(self._scene_object_ids) | set(self._attached_ids)
        if target_co_id:
            object_ids.add(target_co_id)

        # 确保所有相关名称都在矩阵中（缺失则扩展方阵，新条目默认禁止碰撞）
        for name in list(self.acm_gripper_links) + sorted(object_ids):
            if name not in names:
                names.append(name)
                for row in values:
                    row.append(False)
                values.append([False] * len(names))

        index = {name: idx for idx, name in enumerate(names)}
        allowed_pairs = []
        for obj_id in object_ids:
            allow = bool(target_co_id) and obj_id == target_co_id
            j = index[obj_id]
            for link in self.acm_gripper_links:
                i = index[link]
                values[i][j] = allow
                values[j][i] = allow
                if allow:
                    allowed_pairs.append(f'{link}<->{obj_id}')

        matrix = AllowedCollisionMatrix()
        matrix.entry_names = names
        for row in values:
            entry = AllowedCollisionEntry()
            entry.enabled = [bool(value) for value in row]
            matrix.entry_values.append(entry)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        if not self._apply_scene(scene):
            self.get_logger().warn('[ACM] 碰撞矩阵写回失败')
            return False
        if allowed_pairs:
            self.get_logger().info(f'[ACM] 豁免夹爪-目标接触: {", ".join(allowed_pairs)}')
        else:
            self.get_logger().info('[ACM] 已恢复基线：夹爪对所有物体禁止碰撞')
        return True

    def _recover_from_invalid_start(self, force: bool = False) -> bool:
        """起点非法（当前关节状态即碰撞）时的强制复位，打断"规划级联失败"。

        现象（本项目实测）：某次抓取失败后，机械臂停在贴着物体/桌面的位姿；此后每轮
        "移到观察位"以及每个候选的预抓取规划都从**碰撞起点**出发，RRTConnect 直接判
        99999，机械臂原地不动，于是下一位姿也从同一非法起点规划 —— 形成级联锁死，
        整轮/整场几乎全部规划失败（探针日志表现为大量"规划起点非法：panda_hand<->..."）。

        对策：检测到起点非法时，**临时把整张 ACM 置为允许碰撞**（读全量后写回，避免破坏
        矩阵结构），据此把机械臂强制规划回观察位（失败再退回 home），随后无论如何都恢复
        基线 ACM。物体/桌面的碰撞体保持不动，复位后由 _sync_object_collision_objects 继续同步。

        返回 True 表示"确实作了复位"；False 表示起点本就合法（或服务不可用）无需处理。
        """
        # 先无条件清掉"幽灵附着"（见 _clear_all_attached）：若撤销附着失败过，MoveIt 会把
        # 物体一直挂在夹爪上，导致起点/轨迹被判碰撞，机械臂卡死——这是本次卡住的根因。
        self._clear_all_attached()
        validity = self.moveit.check_state_validity(self.joint_names)
        if not force and (validity is None or validity.get('valid')):
            return False  # 起点本就合法 -> 无需复位（force=True 时强制走一次逃逸）

        contacts = '; '.join(validity.get('contacts', [])) or '（无接触明细）'
        self.get_logger().warn(f'[恢复] 规划起点非法（起点即碰撞），执行强制复位：{contacts}')

        current = self.moveit.get_allowed_collision_matrix()
        if current is None:
            self.get_logger().warn('[恢复] 无法读取碰撞矩阵，跳过强制复位')
            return False
        names, _values = current

        matrix = AllowedCollisionMatrix()
        matrix.entry_names = list(names)
        for _ in names:
            entry = AllowedCollisionEntry()
            entry.enabled = [True] * len(names)
            matrix.entry_values.append(entry)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        if not self._apply_scene(scene):
            self.get_logger().warn('[恢复] 临时放开碰撞矩阵失败，跳过强制复位')
            return False

        try:
            target = self.observe_joint_values or self.home_joint_values
            ok, message = self.moveit.move_to_joint_values(
                target, self.velocity, self.acceleration, timeout=30.0)
            if not ok and self.home_joint_values and list(self.home_joint_values) != list(target):
                ok, message = self.moveit.move_to_joint_values(
                    self.home_joint_values, self.velocity, self.acceleration, timeout=30.0)
            self.get_logger().warn(f'[恢复] 强制复位结果: {ok}（{message}）')
            return bool(ok)
        finally:
            # 恢复基线 ACM（夹爪/手臂对所有物体禁止碰撞），确保后续正常规划
            self._acm_set_target(None)

    def _on_clusters(self, msg: MarkerArray):
        """缓存聚类质心与尺寸（CUBE marker：位姿=质心，scale=包围盒尺寸）。"""
        clusters = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE and marker.action == Marker.ADD:
                centroid = np.array([marker.pose.position.x, marker.pose.position.y,
                                     marker.pose.position.z])
                extent = np.array([marker.scale.x, marker.scale.y, marker.scale.z])
                clusters.append((centroid, extent))
        if clusters:
            self._clusters = clusters

    def _nearest_cluster(self, center):
        """返回离抓取中心最近的簇 (质心, 尺寸, 距离)；无聚类数据时返回 None。"""
        if not self._clusters:
            return None
        best = None
        for centroid, extent in self._clusters:
            dist = float(np.linalg.norm(center - centroid))
            if best is None or dist < best[2]:
                best = (centroid, extent, dist)
        return best

    def _log_candidate_diagnostic(self, grasp, grasp_pose, approach, tcp_info=None):
        """候选定位日志：一次判定空抓是"横向偏了"还是"下压不足/过深"。

        打印内容：评分/开口、接近倾角、抓取中心、与最近簇质心的距离与水平偏差、
        该簇包围盒尺寸，以及按真实 TCP_LENGTH 换算的指尖落点高度与距桌面余量。
        指尖余量为负或过小 => 下压会被桌面碰撞体拦截（应增大 tcp_offset）；
        中心离最近簇水平偏差大 => 指尖落到物体之外（横向偏，应弃用该候选）。
        """
        center = np.array([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
        link8 = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
        tip = link8 + np.asarray(approach, dtype=float) * self.tcp_length
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, -float(approach[2])))))
        tip_margin = float(tip[2] - self.table_top_z)

        nearest = self._nearest_cluster(center)
        if nearest is None:
            cluster_text = '最近簇=无数据'
        else:
            centroid, extent, dist = nearest
            horizontal = float(np.linalg.norm(center[:2] - centroid[:2]))
            cluster_text = (
                f'最近簇 距离={dist * 1000:.0f}mm(水平{horizontal * 1000:.0f}mm) '
                f'尺寸={extent[0] * 1000:.0f}×{extent[1] * 1000:.0f}×{extent[2] * 1000:.0f}mm'
            )

        self.get_logger().info(
            f'[诊断] score={float(grasp.score):.3f} 开口={float(grasp.width) * 1000:.0f}mm '
            f'倾角={tilt_deg:.0f}° | 抓取中心=({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) | '
            f'{cluster_text} | link8目标=({link8[0]:.3f},{link8[1]:.3f},{link8[2]:.3f}) | '
            f'指尖目标z={tip[2] * 1000:.0f}mm 距桌面={tip_margin * 1000:.1f}mm')

        if tcp_info is not None:
            tcp_eff, tcp_min, min_corner_z = tcp_info
            raised = tcp_eff - self.tcp_offset
            lower_text = '—' if tcp_min is None else f'{tcp_min:.3f}'
            self.get_logger().info(
                f'[诊断] tcp_offset 名义={self.tcp_offset:.3f} 实际={tcp_eff:.3f}'
                f'（抬升 {raised * 1000:+.1f}mm，桌面间隙下限={lower_text}）'
                f' | 手指盒最低角点 z={min_corner_z * 1000:.0f}mm'
                f'（距桌面 {(min_corner_z - self.table_top_z) * 1000:.1f}mm）'
                f'{" [已按桌面间隙抬升]" if raised > 1e-4 else ""}')

        if tip_margin < 0.002:
            self.get_logger().warn(
                f'[诊断] 指尖目标距桌面仅 {tip_margin * 1000:.1f}mm（<2mm），'
                f'直线下压大概率被桌面碰撞体拦截 -> 需增大 tcp_offset 或换候选')

    def _log_pregrasp_probe(self, pregrasp_pose, grasp_pose):
        """定位"到预抓取点"位姿规划失败（MoveIt 99999）的根因。

        move_group 日志中的 "Unable to sample any valid states for goal tree" 只对应三种可能：
            1) 起点本身非法（自碰撞 / 超出关节限位）；
            2) 目标位姿根本没有逆解（关节限位 / 姿态约束太紧）；
            3) 目标位姿有逆解，但任何解都必然碰撞。
        本探针依次调用 /check_state_validity（当前状态）与 /compute_ik（预抓取位姿、
        抓取位姿，各做"含/不含碰撞检测"两次），把三者区分开，且**不产生任何运动**。
        """
        p, q = pregrasp_pose.position, pregrasp_pose.orientation
        self.get_logger().info(
            f'[探针] 预抓取位姿 link8=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
            f'quat=({q.x:+.3f},{q.y:+.3f},{q.z:+.3f},{q.w:+.3f}) '
            f'| 沿 approach 后退 {self.pregrasp_retreat * 1000:.0f}mm')

        validity = self.moveit.check_state_validity(self.joint_names)
        if validity is None:
            self.get_logger().warn('[探针] /check_state_validity 不可用，跳过起点检查')
        elif validity['valid']:
            self.get_logger().info('[探针] 规划起点（当前状态）合法：无碰撞')
        else:
            self.get_logger().warn(
                f'[探针] 规划起点非法（起点即碰撞，RRTConnect 会直接判 99999）：'
                f'{"; ".join(validity["contacts"]) or "（无接触明细）"}')

        ik_free = {}
        for tag, pose in (('预抓取', pregrasp_pose), ('抓取', grasp_pose)):
            free = self.moveit.compute_ik(pose, avoid_collisions=False)
            safe = self.moveit.compute_ik(pose, avoid_collisions=True)
            ik_free[tag] = free is not None and free['error'] == 1
            self.get_logger().info(
                f'[探针] {tag}位姿 IK：仅运动学={_ik_text(free)} | 运动学+无碰撞={_ik_text(safe)}')
            # IK 响应不带接触信息；若"有解但必然碰撞"，用该解反查碰撞对，指出撞到谁。
            if free is not None and free['error'] == 1 and (safe is None or safe['error'] != 1):
                detail = self.moveit.check_state_validity(free['joint_names'], free['positions'])
                if detail is not None and not detail['valid']:
                    self.get_logger().warn(
                        f'[探针] {tag}位姿的运动学解碰撞于：'
                        f'{"; ".join(detail["contacts"]) or "（无接触明细）"}')

        if ik_free.get('预抓取') and ik_free.get('抓取'):
            self.get_logger().warn(
                '[探针] 两个位姿都存在无碰撞逆解 -> 起点合法且目标可达，'
                '99999 属 OMPL 偶发采样失败（应提高 planning_attempts 或换规划器）')
        elif not ik_free.get('预抓取'):
            self.get_logger().warn(
                '[探针] 预抓取位姿无逆解 -> 失败在"目标位姿不可达"（关节限位/姿态约束），'
                '与起点无关，应缩短 pregrasp_retreat 或放宽姿态约束')
        else:
            self.get_logger().warn(
                '[探针] 抓取位姿无逆解但预抓取位姿可行 -> 下压段不可达（属 Step 2 范畴）')

    def _log_cartesian_probe(self, grasp_pose):
        """在预抓取位上"空算"下压路径（不执行），定位失败原因：碰撞 vs 关节限位/逆解。

        含碰撞比例明显低于忽略碰撞、且忽略碰撞≈100% => 被碰撞体拦截；
        两者都明显偏低 => 关节限位或逆解不可达。
        """
        frac_c, _ = self.moveit.compute_cartesian_fraction([grasp_pose], avoid_collisions=True)
        frac_nc, _ = self.moveit.compute_cartesian_fraction([grasp_pose], avoid_collisions=False)
        self.get_logger().info(
            f'[诊断] 下压路径可规划比例：含碰撞检测={_fmt_frac(frac_c)} '
            f'忽略碰撞={_fmt_frac(frac_nc)}')
        if frac_c is not None and frac_c < 0.99:
            report = self.moveit.cartesian_collision_report([grasp_pose], self.joint_names)
            if report:
                self.get_logger().warn(
                    f'[诊断] 首次碰撞发生于下压 {_fmt_frac(report["at"])} 处，接触: '
                    f'{"; ".join(report["contacts"]) or "（无接触）"}')
                ee_pose = report.get('ee_pose')
                finger_pose = report.get('finger_pose')
                if ee_pose is not None:
                    r = _quat_to_matrix(ee_pose[3], ee_pose[4], ee_pose[5], ee_pose[6])
                    tip_z = ee_pose[2] + float(r[2, 2]) * self.tcp_length
                    self.get_logger().warn(
                        f'[诊断] 实测 link8 z={ee_pose[2] * 1000:.0f}mm，'
                        f'折算指尖 z={tip_z * 1000:.0f}mm（距桌面 {tip_z * 1000:.0f}mm）')
                if finger_pose is not None:
                    self.get_logger().warn(
                        f'[诊断] 实测 panda_leftfinger 原点 z={finger_pose[2] * 1000:.0f}mm')
        if frac_c is not None and frac_nc is not None and frac_nc > 0.99 and frac_c < 0.99:
            self.get_logger().warn(
                '[诊断] 忽略碰撞即可全程下压 -> 拦截来自碰撞体（手指/桌面/自身），非关节限位')
        elif frac_c is not None and frac_c < 0.99:
            self.get_logger().warn(
                '[诊断] 忽略碰撞后仍无法全程下压 -> 关节限位或逆解不可达，需换候选')

    def _wait_fresh_grasps(self, timeout: float = 5.0):
        seq0 = self._grasp_seq
        deadline = time.time() + timeout
        while time.time() < deadline and self._grasp_seq <= seq0 and rclpy.ok():
            time.sleep(0.05)
        if self._grasp_seq <= seq0:
            return None
        return self._latest_grasps

    def _set_state(self, state: str):
        self._state = state
        message = String()
        message.data = state
        self.pub_status.publish(message)
        self.get_logger().info(f'[状态] {state}')

    def _publish_statistics(self):
        message = SortingStatistics()
        message.header.stamp = self.get_clock().now().to_msg()
        message.attempted = int(self.stats['attempted'])
        message.picked_success = int(self.stats['picked_success'])
        message.grasp_failures = int(self.stats['grasp_failures'])
        message.retries = int(self.stats['retries'])
        message.grasp_success_rate = (
            float(message.picked_success) / message.attempted if message.attempted else 0.0
        )
        message.avg_cycle_time = float(np.mean(self._cycle_times)) if self._cycle_times else 0.0
        message.state = self._state
        self.pub_statistics.publish(message)

    def _emit_eval(self):
        """输出一行可解析的评估指标（批量实验脚本据此逐轮抓取数据）。

        覆盖三层指标所需的原始计数：规划请求/成功（规划成功率）、空抓与抓取确认
        （空抓率）、抓取确认与成功搬运（放置成功率）、尝试与成功（端到端成功率）。
        """
        plan_now = self.moveit.plan_stats()
        base = self._plan_baseline
        close_attempts = self.stats['empty_grasps'] + self.stats['grasp_confirmed']
        payload = {
            'attempted': self.stats['attempted'],
            'picked_success': self.stats['picked_success'],
            'grasp_failures': self.stats['grasp_failures'],
            'plan_failures': self.stats['plan_failures'],
            'empty_grasps': self.stats['empty_grasps'],
            'grasp_confirmed': self.stats['grasp_confirmed'],
            'close_attempts': close_attempts,
            'lift_slip': self.stats['lift_slip'],
            'drop_plan_fail': self.stats['drop_plan_fail'],
            'plan_requests': plan_now['plan_requests'] - base['plan_requests'],
            'plan_successes': plan_now['plan_successes'] - base['plan_successes'],
            'avg_cycle_time': round(float(np.mean(self._cycle_times)), 3) if self._cycle_times else 0.0,
        }
        self.get_logger().info('[EVAL] ' + json.dumps(payload))

    # ==================================================================
    # 抓取姿态映射
    # ==================================================================
    def grasp_to_pose(self, grasp):
        """Grasp6D（world 系）-> panda_link8 末端位姿（world 系 == base 系）。

        返回 (pose, approach, tcp_eff)，其中 tcp_eff 为本候选实际使用的 tcp_offset。

        逐候选 tcp_offset（Step1）：全局常量无法同时满足"偏低插桌"与"偏高空抓"。
        这里按候选姿态精确计算手指碰撞盒 16 个角点中最低者的世界 z，并解出使该最低点
        恰好落在 ``桌面 + table_clearance_margin`` 之上的最小 tcp_offset，再与名义值取
        较大者（名义值已足够安全时保持不变），并夹在 [名义值, tcp_offset_max] 内。
        """
        r_grasp = _quat_to_matrix(
            grasp.pose.orientation.x, grasp.pose.orientation.y,
            grasp.pose.orientation.z, grasp.pose.orientation.w,
        )
        approach = r_grasp[:, 0]
        closing = r_grasp[:, 1]
        r_link8 = link8_orientation(approach, closing)

        center = np.array([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])

        # 各角点相对 link8 的世界 z 偏移（取最小值 = 最低角点）
        corner_dz = (r_link8 @ self._finger_corners_link8.T)[2, :]
        min_corner_dz = float(corner_dz.min())
        # 解 link8_z + min_corner_dz >= 桌面 + 余量，且 link8_z = center_z - approach_z*tcp
        approach_z = float(approach[2])
        if approach_z < -1e-6:
            tcp_min = (self.table_top_z + self.table_clearance_margin
                       - center[2] - min_corner_dz) / (-approach_z)
            tcp_eff = min(max(self.tcp_offset, tcp_min), self.tcp_offset_max)
        else:  # 非向下接近（理论上已被 planner 过滤），保守回退名义值
            tcp_eff = self.tcp_offset

        position = center - approach * tcp_eff
        min_corner_world_z = float(position[2] + min_corner_dz)

        qx, qy, qz, qw = _matrix_to_quat(r_link8)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation.x, pose.orientation.y = float(qx), float(qy)
        pose.orientation.z, pose.orientation.w = float(qz), float(qw)
        return pose, approach, (tcp_eff, tcp_min if approach_z < -1e-6 else None,
                                min_corner_world_z)

    def _publish_execution_markers(self, grasp_pose, grasp):
        """发布"抓取中心(红) vs 实际下发的 link8 位姿(蓝)"，供 rviz 复核 tcp_offset 包裹效果。

        红球为 GraspNet 给出的抓取中心；蓝箭头为补偿 tcp_offset 后真正下发的末端位姿。
        两者沿 approach 相距 tcp_offset，指向即接近方向。
        """
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        center = Marker()
        center.header.frame_id = 'world'
        center.ns = 'grasp_center'
        center.id = 0
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.pose = grasp.pose
        center.scale.x = center.scale.y = center.scale.z = 0.02
        center.color.r, center.color.g, center.color.b, center.color.a = 0.95, 0.2, 0.2, 1.0
        arr.markers.append(center)

        ee = Marker()
        ee.header.frame_id = 'world'
        ee.ns = 'ee_target'
        ee.id = 1
        ee.type = Marker.ARROW
        ee.action = Marker.ADD
        ee.pose = grasp_pose
        ee.scale.x, ee.scale.y = 0.10, 0.010
        ee.color.r, ee.color.g, ee.color.b, ee.color.a = 0.2, 0.5, 0.95, 1.0
        arr.markers.append(ee)
        self.pub_markers.publish(arr)

    # ==================================================================
    # 服务
    # ==================================================================
    def on_start_sorting(self, request, response):
        if self._running:
            response.success = False
            response.message = '抓取任务已在运行中'
            return response
        if not self._bootstrapped:
            response.success = False
            response.message = '执行节点尚未就绪（等待 MoveIt / 场景初始化）'
            return response

        continuous = bool(request.continuous)
        for key in self.stats:
            self.stats[key] = 0
        # 规划请求计数基线：EVAL 只统计本轮抓取任务期间发出的规划请求
        self._plan_baseline = self.moveit.plan_stats()
        self._cycle_times.clear()
        self._running = True
        import threading
        self._worker = threading.Thread(target=self._run_loop, args=(continuous,), daemon=True)
        self._worker.start()
        response.success = True
        response.message = f'GraspNet 抓取任务已启动（{"连续" if continuous else "单次"}模式）'
        return response

    # ==================================================================
    # 主循环
    # ==================================================================
    def _return_home(self):
        """任务结束时把机械臂送回初始位（home），避免停在半空/工作区。

        此前没有这一步：任务 DONE 后就停在原地（若"移到观察位"还失败过，看着就像卡死）。
        """
        self._acm_set_target(None)
        self._clear_all_attached()   # 先清幽灵附着，否则夹爪上的幽灵会让回位规划也 -2
        targets = []
        if self.home_joint_values:
            targets.append(('初始位', self.home_joint_values))
        if (self.observe_joint_values
                and list(self.observe_joint_values) != list(self.home_joint_values)):
            targets.append(('观察位', self.observe_joint_values))
        for name, values in targets:
            ok, message = self.moveit.move_to_joint_values(
                values, self.velocity, self.acceleration, timeout=30.0)
            if ok:
                self.get_logger().info(f'任务结束：已回到{name}')
                return True
            self.get_logger().warn(f'任务结束回到{name}失败（{message}）')
        self._recover_from_invalid_start(force=True)
        return False

    def _log_no_attemptable(self, grasps_msg):
        """诊断：attempted==0 时逐组说明为何不可尝试（黑名单 / 包络外 / 已抓取）。"""
        groups = self._group_grasps(grasps_msg.grasps)
        n_black = n_env = 0
        for index, group in enumerate(groups):
            c = group['center']
            if self._is_blacklisted(c):
                n_black += 1
                self.get_logger().info(
                    f'[诊断] 物体{chr(ord("A") + index)} 中心({c[0]:.3f},{c[1]:.3f}) 在【黑名单】中，跳过')
            elif not self._in_grasp_envelope(c[0], c[1]):
                n_env += 1
                self.get_logger().info(
                    f'[诊断] 物体{chr(ord("A") + index)} 中心({c[0]:.3f},{c[1]:.3f}) 在【可抓包络外】，跳过')
        remaining = [n for n, (p, _s, g) in self._gt_objects.items()
                     if not g and self._in_grasp_envelope(p.position.x, p.position.y)]
        self.get_logger().warn(
            f'[诊断] 无可尝试物体：候选组 {len(groups)}（黑名单 {n_black} / 包络外 {n_env}）；'
            f'包络内未抓取真值物体 {len(remaining)} 个')

    def _log_scene_mismatch(self):
        """诊断：把 move_group **实际的**规划场景内容打印出来，与本节点账本对比。

        为什么需要：本项目出现过"物体早已掉落、场景里却仍有悬空碰撞盒"的恶性问题，
        它会把后续规划打成 -2。仅靠节点自己的账本无法发现（那些盒子不在账本里），
        所以直接问 move_group：世界系有哪些物体、分别在哪、夹爪上附着着什么。
        """
        result = self.moveit.get_scene_objects(timeout=5.0)
        if result is None:
            self.get_logger().warn('[诊断] /get_planning_scene 不可用，无法核对规划场景')
            return
        world, attached = result
        self.get_logger().warn(
            f'[诊断] move_group 实际场景：附着物体={attached}；世界物体={[w[0] for w in world]}')
        for obj_id, x, y, z in world:
            # 只有 pick_* 归本节点管；bin_zone_*/work_table 是静态环境体，不算异常
            if not obj_id.startswith('pick_'):
                self.get_logger().info(f'[诊断] 环境物体 {obj_id}')
                continue
            mark = '  <<< 账本外（悬空盒嫌疑）' if obj_id not in self._scene_object_ids else ''
            self.get_logger().warn(
                f'[诊断] 待抓物体碰撞盒 {obj_id} @ ({x:.3f},{y:.3f},{z:.3f}){mark}')

    def _park_for_detection(self):
        """DETECT 前把机械臂移出工作区（观察位），避免相机把机械臂本体拍进去。

        为什么必须做：机械臂只要停在工作区（例如失败后停在预抓取位 (0.308,-0.085,0.246)），
        它自己就会被点云聚成一个"物体"，GraspNet 随即在手臂上提议抓取——实测产生过
        抓取中心 z=0.217m 的"空中假目标"（比桌面高 200mm），下压时规划层看那里"空无一物"
        于是顺利执行，闭合后必然空抓，白白消耗尝试次数并污染候选池。
        停到观察位后工作区视野干净，候选只来自真实物体。
        """
        if not self.observe_joint_values:
            return
        ok, message = self.moveit.move_to_joint_values(
            self.observe_joint_values, self.velocity, self.acceleration, timeout=30.0
        )
        if not ok:
            self.get_logger().warn(f'移到观察位失败（{message}），继续尝试检测')

    def _run_loop(self, continuous: bool):
        empty_rounds = 0
        self._blacklist = []
        try:
            while self._running and rclpy.ok():
                self._set_state('DETECT')
                # 每轮开始清掉"物体被碰移"标记（重新感知后重新判定）
                self._object_disturbed = False
                # 先把规划场景与真值对齐：这会清掉上一轮遗留的"悬空/幽灵"碰撞盒。
                # 必须放在复位/停车**之前**——否则陈旧的碰撞盒会让"复位/移到观察位"
                # 的规划直接判 -2，而能清掉它的同步却排在最后，等于永远解不开。
                self._sync_object_collision_objects()
                # 若上一轮遗留了非法起点（机械臂停靠在物体/桌面上），先强制复位再停车；
                # 否则"移到观察位"的规划会从碰撞起点出发直接 99999，触发整轮级联失败。
                self._recover_from_invalid_start()
                # 再把机械臂挪出工作区（否则它会被拍进点云、GraspNet 在手臂上提议抓取）
                self._park_for_detection()
                grasps = self._wait_fresh_grasps(timeout=8.0)
                if grasps is None or not grasps.grasps:
                    empty_rounds += 1
                    if empty_rounds >= self.max_empty_rounds:
                        self.get_logger().info(
                            f'连续 {empty_rounds} 轮未获取到 GraspNet 抓取候选，任务结束')
                        break
                    self.get_logger().warn(
                        f'本轮未获取到抓取候选，2s 后重试（{empty_rounds}/{self.max_empty_rounds}）')
                    time.sleep(2.0)
                    continue

                success, attempted = self._attempt_grasps(grasps)
                self._emit_eval()
                empty_rounds = 0 if success else empty_rounds + 1
                if not continuous:
                    break
                if success:
                    continue
                if attempted == 0:
                    # 逐组说明"为什么没有可尝试物体"（黑名单 / 包络外），避免含糊结论
                    self._log_no_attemptable(grasps)
                    # 无可用物体（全部在黑名单 / 已被抓走）：等下一帧，连续超限即收工
                    if empty_rounds >= self.max_empty_rounds:
                        self.get_logger().info('已无可尝试物体（全部黑名单或已抓取），任务结束')
                        break
                    time.sleep(2.0)
                else:
                    self.get_logger().info('本轮抓取失败，进入下一轮')
            # 任务结束：把机械臂送回初始位（此前缺失，导致任务结束后停在半空/工作区）
            self._return_home()
            self._log_scene_mismatch()
            self._emit_eval()
            self._set_state('DONE')
        except Exception as exc:
            self.get_logger().error(f'抓取循环异常: {exc}')
            self._set_state('IDLE')
        finally:
            self._running = False

    def _group_grasps(self, grasps):
        """把无物体 ID 的抓取候选按空间邻近聚成"物体组"（组内按评分降序）。

        GraspNet 只输出抓取候选、不带物体 ID；必须先按 3cm 邻近把同一物体上的候选归组，
        才能实现"单物体尝试上限 + 黑名单"（否则系统会在候选间全局横跳）。
        """
        groups = []
        for grasp in sorted(grasps, key=lambda g: -float(g.score)):
            center = np.array([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
            for group in groups:
                if float(np.linalg.norm(center - group['center'])) <= self.object_cluster_radius:
                    group['grasps'].append(grasp)
                    break
            else:
                groups.append({'center': center.copy(), 'grasps': [grasp]})
        groups.sort(key=lambda group: -float(group['grasps'][0].score))
        return groups

    def _is_blacklisted(self, center) -> bool:
        for blocked in self._blacklist:
            if float(np.linalg.norm(center - blocked)) <= self.object_cluster_radius:
                return True
        return False

    def _in_grasp_envelope(self, x: float, y: float) -> bool:
        """物体/抓取中心是否落在"可抓包络"内（world 系）。包络外 = 离场，不参与抓取。"""
        x_min, x_max, y_min, y_max = self.grasp_envelope
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    def _screen_candidates(self, grasps):
        """对每组前 N 个候选做"无碰撞逆解"预检，可行候选优先返回。

        物体靠得近时，最高分候选的抓取位姿/下压路径常被邻物挡住，MoveIt 会判碰撞失败；
        若只按分数取前 2 个候选，很可能两个都被挡，于是误判该物体不可抓并拉黑。
        这里对前 candidate_screen_topk 个候选逐一算"避障逆解"：可解的排到前面优先尝试，
        无解的排后面兜底，从而在物体密集时也能挑到真正可达的候选。
        """
        head = list(grasps[:self.candidate_screen_topk])
        feasible, infeasible = [], []
        for grasp in head:
            try:
                grasp_pose, _approach, _info = self.grasp_to_pose(grasp)
            except Exception:
                infeasible.append(grasp)
                continue
            # 预筛只做纯运动学可达性判断（不引入碰撞）：此时尚未确定目标物体、
            # ACM 豁免还没建立，"含碰撞"会把"可抓但手指接触目标"的候选全部误杀。
            # 真正的避障由下压规划在 ACM 就绪后判断。
            ik = self.moveit.compute_ik(grasp_pose, avoid_collisions=False)
            if ik is None:
                # 逆解服务不可用：不做筛查，退回原始顺序，避免误伤
                return list(grasps)
            (feasible if ik.get('error') == 1 else infeasible).append(grasp)
        return feasible + infeasible + list(grasps[self.candidate_screen_topk:])

    def _attempt_grasps(self, grasps_msg):
        """按"物体组"遍历抓取：组内最多尝试 max_attempts_per_object 个候选，全失败入黑名单。

        返回 (是否成功, 本轮实际尝试的候选数)。attempted=0 表示当前无可用物体（全在黑名单）。
        """
        groups = self._group_grasps(grasps_msg.grasps)
        # 抓取顺序不再强制"从高到低"：沿用 _group_grasps 的排序（按 GraspNet 候选评分降序），
        # 由系统按抓取质量自行决定先抓谁后抓谁。
        attempted = 0
        for index, group in enumerate(groups):
            label = f'物体{chr(ord("A") + index)}'
            if self._is_blacklisted(group['center']):
                continue
            if not self._in_grasp_envelope(group['center'][0], group['center'][1]):
                self.get_logger().info(
                    f'跳过{label}：中心 ({group["center"][0]:.3f},{group["center"][1]:.3f}) '
                    '在可抓包络之外，不尝试抓取')
                continue
            ordered = self._screen_candidates(group['grasps'])
            candidates = ordered[:self.max_attempts_per_object]
            self.get_logger().info(
                f'尝试抓取{label}（中心 {group["center"][0]:.3f},{group["center"][1]:.3f}），'
                f'候选 {len(candidates)}/{len(group["grasps"])}')
            for i, grasp in enumerate(candidates, 1):
                self._set_state('EXECUTE')
                attempted += 1
                ok, message = self._execute_one(grasp)
                if ok:
                    self.get_logger().info(
                        f'{label} 抓取成功（候选 {i}/{len(candidates)}，score={grasp.score:.3f}）')
                    # 成功搬走一个物体后，之前因"邻物挡路"被判不可抓而拉黑的物体
                    # 可能已重新可达，清空黑名单给它们下一轮重试的机会。
                    self._blacklist = []
                    return True, attempted
                self.get_logger().warn(f'{label} 候选{i} 失败，原因：{message}')
                if self._object_disturbed:
                    # 空抓/掉落说明很可能把物体碰移了；继续用同一张旧快照的剩余候选，
                    # 只会对着旧位置反复下压。立即结束本轮、重新感知后再试。
                    self.get_logger().warn(
                        f'{label} 疑似把物体碰移（{message}）；放弃本轮剩余候选，'
                        '重新感知后重试，避免继续对着旧位置下压')
                    return False, attempted
            self._blacklist.append(group['center'].copy())
            self.get_logger().info(
                f'{label} 尝试次数超限（{len(candidates)}/{len(group["grasps"])}），加入黑名单')
        return False, attempted

    def _execute_one(self, grasp):
        """执行一次抓取；无论成功/失败，退出时都把 ACM 恢复基线（见 finally）。"""
        try:
            return self._execute_one_inner(grasp, time.time())
        finally:
            # 每轮抓取结束（含中途失败/异常）都清掉夹爪-目标豁免，保证下一轮 ACM 干净
            self._acm_set_target(None)

    def _execute_one_inner(self, grasp, start):
        """执行一次"预抓取->下降->闭合->抬升->搬运->释放"。返回 (是否成功, 说明)。"""
        try:
            grasp_pose, approach, tcp_info = self.grasp_to_pose(grasp)
        except Exception as exc:
            return False, f'姿态映射失败: {exc}'

        self._publish_execution_markers(grasp_pose, grasp)
        self._log_candidate_diagnostic(grasp, grasp_pose, approach, tcp_info)

        # 选定本轮目标物体（离抓取中心最近的真值物体），规划前据此刷新 ACM：
        # 只豁免"夹爪连杆 <-> 该物体"，其余物体/桌面/手臂一律保持严格。
        object_name = self._nearest_gt_object(
            (grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z))
        # 起点若非法（上一候选遗留的碰撞位姿）先强制复位，否则本候选必然 99999。
        self._recover_from_invalid_start()
        self._acm_set_target(f'pick_{object_name}' if object_name else None)

        pregrasp_pose = copy.deepcopy(grasp_pose)
        pregrasp_pose.position.x -= approach[0] * self.pregrasp_retreat
        pregrasp_pose.position.y -= approach[1] * self.pregrasp_retreat
        pregrasp_pose.position.z -= approach[2] * self.pregrasp_retreat

        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.x -= approach[0] * self.lift_retreat
        lift_pose.position.y -= approach[1] * self.lift_retreat
        lift_pose.position.z -= approach[2] * self.lift_retreat

        self.stats['attempted'] += 1
        self.gripper.open()

        # 1) 到预抓取点（位姿目标规划）
        #    规划前先跑无副作用探针：把 99999 拆成"起点非法 / 目标无逆解 / OMPL 偶发"。
        self._log_pregrasp_probe(pregrasp_pose, grasp_pose)
        ok, reason = self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
        if not ok:
            self.stats['plan_failures'] += 1
            return False, f'预抓取点规划失败: {reason}'

        # 1b) 诊断（不执行）：在预抓取位上先"空算"下压路径，定位失败是碰撞还是关节限位。
        self._log_cartesian_probe(grasp_pose)

        # 2) 沿 approach 直线下降到抓取位姿（两阶段规划的第二步）
        #    严格禁止"从当前位置直接规划到接触点"：直线插补不可达即放弃该候选，
        #    否则 OMPL 会生成绕行路径撞偏/顶飞物体（这正是"原地空转、来回扭"的来源）。
        #    含碰撞检测（True）：手指与**目标物体**的必然接触已由上面的 ACM 豁免，
        #    其余（桌面/邻物/自碰撞）仍严格拦截。
        ok, reason = self.moveit.move_cartesian(
            [grasp_pose], velocity=self.approach_velocity, max_step=0.003, timeout=30.0,
            avoid_collisions=True
        )
        if not ok:
            self.stats['plan_failures'] += 1
            self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
            return False, f'直线下压不可达（禁止退化为位姿规划）: {reason}'

        # 3) 闭合夹爪并做抓取验证（位置 + 力矩双反馈；闭合到极限即判空抓）
        finger_position = self.gripper.close_and_measure()
        finger_effort = self.gripper.finger_effort()
        grasped = self.gripper.is_grasped(finger_position)
        # 分项计数：闭合确认夹住 = 抓取确认成功；闭合到底未夹住 = 空抓
        if grasped:
            self.stats['grasp_confirmed'] += 1
        else:
            self.stats['empty_grasps'] += 1
        self.get_logger().info(
            f'抓取验证：手指位置={_fmt(finger_position)} 力矩={_fmt(finger_effort)} '
            f'阈值={self.gripper.grasp_threshold} => {"夹住" if grasped else "空抓"}')
        if not grasped:
            # 空抓：原地判失败并退回预抓取点，绝不带着空夹爪移动到投放区
            self.stats['grasp_failures'] += 1
            self._object_disturbed = True
            self.gripper.open()
            self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
            return False, '夹爪闭合到底（空抓），原地判失败'

        self.gripper.hold_closing()

        # 3b) 把刚夹住的物体从世界系挂到夹爪上。
        #     否则规划器会把"手上的物体"与"场景里同一物体的副本"判为碰撞，
        #     抬升轨迹直接被拒（实测 MoveIt -2 INVALID_MOTION_PLAN）。
        attached = bool(object_name) and self._attach_object(object_name)

        # 4) 沿 approach 直线抬升（退离物体）
        ok, reason = self.moveit.move_cartesian(
            [lift_pose], velocity=self.cartesian_velocity, max_step=0.003, timeout=40.0
        )
        if not ok:
            ok, reason = self.moveit.move_to_pose(lift_pose, self.approach_velocity, self.approach_velocity)
        if not ok:
            self.gripper.open()
            if attached:
                self._detach_object(object_name)
            self.stats['grasp_failures'] += 1
            self.stats['lift_slip'] += 1
            self._object_disturbed = True
            return False, f'抬升失败: {reason}'

        # 5) 搬运到投放点 -> 释放（按物体所属颜色分拣区投放）
        drop_pose = self._drop_pose_for(object_name)
        pre_drop = copy.deepcopy(drop_pose)
        pre_drop.position.z += 0.10
        ok, reason = self.moveit.move_to_pose(pre_drop, self.velocity, self.acceleration)
        if not ok:
            self.gripper.open()
            if attached:
                self._detach_object(object_name)
            self.stats['drop_plan_fail'] += 1
            return False, f'投放点上方规划失败: {reason}'
        ok, reason = self.moveit.move_cartesian(
            [drop_pose], velocity=self.cartesian_velocity, max_step=0.004, timeout=30.0
        )
        if not ok:
            self.moveit.move_to_pose(drop_pose, self.approach_velocity, self.approach_velocity)
        self.gripper.release()
        if attached:
            # 撤销附着；世界系副本由下一轮同步按新的真值位姿放回
            self._detach_object(object_name)
        self.moveit.move_cartesian(
            [pre_drop], velocity=self.cartesian_velocity, max_step=0.004, timeout=30.0
        )

        self.stats['picked_success'] += 1
        duration = time.time() - start
        self._cycle_times.append(duration)
        return True, f'抓取并搬运完成，耗时 {duration:.2f}s'


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetPickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()