"""模块4 抓取决策节点。

抓取策略（对应 teach.txt 第八节）：
    * 统一采用"末端垂直向下"的两指夹取，只估计位置 (x, y, z) 与平面内 yaw
    * 对长条物体（elongation 大）使用掩码主轴方向，让夹爪垂直于长轴闭合
    * 输出四段式位姿：pre-grasp -> grasp -> lift -> place

坐标系约定（与 panda_gz.urdf.xacro 一致）：
    抓取位姿定义在 MoveIt 的末端参考连杆 panda_link8 上。
    link8 的 +z 轴指向夹爪前方（指尖方向），垂直向下抓取即让 +z 指向世界 -Z，
    对应姿态四元数 (x=1, y=0, z=0, w=0)，再绕世界 Z 轴叠加 yaw：
        q = q_z(yaw) * q_x(pi)

    由 URDF 可知 panda_hand 相对 panda_link8 绕 z 旋转 -45°，手指沿 panda_hand 的
    ±y 闭合，因此"夹爪闭合轴在世界中的方位角" = yaw - 45°。
    反解得到 yaw = 期望闭合方位角 + pi/4。
"""

import math

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Pose, Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import Detection, GraspCandidate
from sorting_interfaces.srv import PlanGrasp

# 夹爪闭合轴相对 yaw 指令的固定偏置（见模块 docstring）
CLOSING_AXIS_YAW_OFFSET = math.pi / 4.0
# 垂直向下抓取：绕 X 轴旋转 180°
QUAT_DOWN = [1.0, 0.0, 0.0, 0.0]


class GraspPlannerNode(Node):
    """四段式抓取位姿生成。"""

    def __init__(self):
        super().__init__('grasp_planner_node')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        self.grasp_cfg = self.system_config['grasp']
        topics = self.system_config['topics']

        self.declare_parameter(
            'pregrasp_height', float(self.grasp_cfg.get('pregrasp_height', 0.12))
        )
        self.pregrasp_height = float(self.get_parameter('pregrasp_height').value)

        self.lift_height = float(self.grasp_cfg.get('lift_height', 0.15))
        self.tcp_length = float(self.grasp_cfg.get('tcp_length', 0.105))
        self.palm_offset = float(self.grasp_cfg.get('palm_offset', 0.066))
        self.palm_clearance = float(self.grasp_cfg.get('palm_clearance', 0.003))
        self.fingertip_min_margin = float(self.grasp_cfg.get('fingertip_min_margin', 0.006))
        self.max_grasp_width = float(self.grasp_cfg.get('max_grasp_width', 0.078))
        self.bin_base_thickness = float(self.grasp_cfg.get('bin_base_thickness', 0.008))
        self.table_top_z = float(self.system_config['scene']['table']['top_z'])
        self.elongation_threshold = 2.5
        self.max_reach = 0.80

        self.create_service(
            PlanGrasp, self.system_config['services']['plan_grasp'], self.on_plan_grasp
        )
        self.pub_candidates = self.create_publisher(
            GraspCandidate, topics['grasp_candidates'], 5
        )
        self.pub_markers = self.create_publisher(MarkerArray, '/sorting/grasp_markers', 2)

        self.get_logger().info('抓取决策节点启动')

    # ------------------------------------------------------------------
    def _grasp_height(self, z_bottom: float, height: float, palm_offset: float = None) -> float:
        """计算 panda_link8 的抓取高度（米）。

        两个几何约束：
          手掌约束：link8 - palm_offset >= z_bottom + height + palm_clearance
                    （手掌碰撞体不能压到物体顶面，否则会与物体互相穿插）
          指尖约束：link8 - tcp_length   >= z_bottom + fingertip_min_margin
                    （指尖不能扎进桌面）
        取两者中较高者，即可在手掌不碰物体的前提下让手指尽量多地包住物体。
        """
        offset = self.palm_offset if palm_offset is None else palm_offset
        top = z_bottom + height
        palm_limit = top + offset + self.palm_clearance
        fingertip_limit = z_bottom + self.fingertip_min_margin + self.tcp_length
        return max(palm_limit, fingertip_limit)

    def _pose(self, x: float, y: float, z: float, yaw: float) -> Pose:
        quaternion = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_from_euler(0.0, 0.0, yaw), QUAT_DOWN
        )
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return pose

    def plan(self, detection: Detection):
        """由单个检测结果生成抓取候选；返回 (candidate, reason)。"""
        if not detection.has_3d:
            return None, '目标缺少三维定位结果'

        class_id = int(detection.class_id)
        height = self.class_table.height_of(class_id)
        width = self.class_table.grasp_width_of(class_id)

        if width > self.max_grasp_width:
            return None, f'物体宽度 {width:.3f}m 超过夹爪最大开口 {self.max_grasp_width:.3f}m'

        x = float(detection.position_base.x)
        y = float(detection.position_base.y)
        z_top = float(detection.position_base.z)

        # 物体底面：由顶面深度减去标称高度得到（受限场景下的已知 SKU 尺寸假设）
        z_bottom = z_top - height
        if z_bottom < self.table_top_z - 0.01:
            z_bottom = self.table_top_z

        reach = math.sqrt(x * x + y * y + (z_bottom + 0.5 * height) ** 2)
        if reach > self.max_reach:
            return None, f'目标超出工作空间（{reach:.3f}m > {self.max_reach:.3f}m）'

        # ---- 夹爪闭合方位角 ----
        if self.class_table.use_orientation(class_id) and detection.elongation > self.elongation_threshold:
            # 长条物体：夹爪垂直于主轴闭合
            closing_angle = float(detection.yaw_base) + math.pi / 2.0
        else:
            # 对称物体：固定沿基座 X 轴闭合，保证可复现
            closing_angle = 0.0
        yaw_cmd = closing_angle + CLOSING_AXIS_YAW_OFFSET

        # ---- 四段式位姿 ----
        grasp_z = self._grasp_height(z_bottom, height)
        grasp_pose = self._pose(x, y, grasp_z, yaw_cmd)
        pregrasp_pose = self._pose(x, y, grasp_z + self.pregrasp_height, yaw_cmd)
        lift_pose = self._pose(x, y, grasp_z + self.lift_height, yaw_cmd)

        zone = self.class_table.placement_zone_of(class_id)
        zone_pose = self.class_table.zone_pose(zone)
        place_z = self._grasp_height(
            self.bin_base_thickness, height,
            palm_offset=self.palm_offset + 0.010,  # 放入分拣箱时再抬高一点，避开箱壁
        )
        place_pose = self._pose(zone_pose[0], zone_pose[1], place_z, yaw_cmd)

        candidate = GraspCandidate()
        candidate.header = detection.header
        candidate.class_id = class_id
        candidate.class_name = detection.class_name
        candidate.confidence = float(detection.confidence)
        candidate.pregrasp_pose = pregrasp_pose
        candidate.grasp_pose = grasp_pose
        candidate.lift_pose = lift_pose
        candidate.place_pose = place_pose
        candidate.placement_zone = zone

        # ---- 抓取质量评分 ----
        confidence_score = float(np.clip(detection.confidence, 0.0, 1.0))
        depth_score = float(np.clip(1.0 - detection.depth_stddev / 0.02, 0.0, 1.0))
        width_score = float(np.clip(1.0 - width / self.max_grasp_width, 0.0, 1.0))
        reach_score = float(np.clip(1.0 - reach / self.max_reach, 0.0, 1.0))
        candidate.score = float(
            0.35 * confidence_score + 0.25 * depth_score
            + 0.15 * width_score + 0.25 * reach_score
        )

        reason = (
            f'grasp=({x:.3f},{y:.3f},{grasp_z:.3f}) yaw={math.degrees(yaw_cmd):.1f}deg '
            f'zone={zone} score={candidate.score:.2f}'
        )
        return candidate, reason

    # ------------------------------------------------------------------
    def on_plan_grasp(self, request, response):
        candidate, reason = self.plan(request.target)
        if candidate is None:
            response.success = False
            response.message = reason
            self.get_logger().warn(f'抓取规划失败: {reason}')
            return response

        response.success = True
        response.message = reason
        response.candidate = candidate
        self.pub_candidates.publish(candidate)
        self._publish_markers(candidate)
        return response

    def _publish_markers(self, candidate: GraspCandidate):
        markers = MarkerArray()
        specs = [
            ('pregrasp', candidate.pregrasp_pose, (0.2, 0.6, 1.0, 0.9)),
            ('grasp', candidate.grasp_pose, (0.1, 1.0, 0.2, 1.0)),
            ('lift', candidate.lift_pose, (1.0, 0.8, 0.1, 0.9)),
            ('place', candidate.place_pose, (1.0, 0.2, 0.5, 0.9)),
        ]
        for index, (name, pose, rgba) in enumerate(specs):
            marker = Marker()
            marker.header = candidate.header
            marker.ns = name
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.08
            marker.scale.y = 0.012
            marker.scale.z = 0.012
            marker.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            markers.markers.append(marker)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()