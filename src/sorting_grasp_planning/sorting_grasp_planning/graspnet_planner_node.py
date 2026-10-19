"""模块4b 6DoF 抓取检测节点（GraspNet 后端）。

职责：
    订阅 sorting_perception 输出的工作区点云（相机光学系），
    用预训练 GraspNet 推理 6DoF 抓取位姿，做宽度/评分过滤，
    转换到 world 系后发布（供任务规划消费 + rviz 可视化）。

关于坐标系：
    * 输入点云必须是**相机光学系**（z 前 / x 右 / y 下）——这是 GraspNet 的训练约定；
      若喂 world 系点云，网络看到的几何分布与训练时不一致，输出会退化。
    * 网络输出的抓取中心/姿态都在相机光学系，本节点用相机外参转到 world 系。

关于过滤：
    * Gate3 实测 83 个抓取中有 26 个宽度超过夹爪 78mm 开口，必须按宽度筛除。
    * 碰撞检测（GraspNet 原版依赖 open3d）在本环境未启用，见 README 的待办说明。
"""

import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from sorting_bringup import camera_extrinsics, load_system_config
from sorting_interfaces.msg import Grasp6D, Grasp6DArray


def _matrix_to_quaternion(m: np.ndarray):
    """3x3 旋转矩阵 -> (x, y, z, w) 四元数。"""
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s)
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s)


class GraspNetPlannerNode(Node):
    """基于预训练 GraspNet 的 6DoF 抓取检测。"""

    def __init__(self):
        super().__init__('graspnet_planner_node')

        cfg = load_system_config()
        gcfg = cfg.get('graspnet', {})
        topics = cfg['topics']

        # ---- 相机外参（唯一真值源见 sorting_bringup.camera，含完整 RPY）----
        self.t_world_cam, self.r_world_optical = camera_extrinsics(cfg)

        # ---- 第三方代码与权重 ----
        repo_root = os.environ.get('GRASPNET_REPO') or gcfg.get('repo_root', '')
        if not repo_root or not os.path.isdir(repo_root):
            self.get_logger().fatal(
                'GraspNet 仓库路径无效: %r（请设置 system.yaml 的 graspnet.repo_root '
                '或环境变量 GRASPNET_REPO）' % repo_root)
            raise RuntimeError('graspnet repo root not found')
        for sub in ('models', 'dataset', 'utils', 'knn', ''):
            p = os.path.join(repo_root, sub)
            if p not in sys.path:
                sys.path.append(p)

        import torch  # noqa: E402
        from graspnet import GraspNet, pred_decode  # noqa: E402
        self.torch = torch
        self.pred_decode = pred_decode

        self.num_point = int(gcfg.get('num_point', 20000))
        self.score_threshold = float(gcfg.get('score_threshold', 0.30))
        self.max_grasps = int(gcfg.get('max_grasps', 30))
        self.max_approach_deg = float(gcfg.get('max_approach_deg', 75.0))
        self.accumulate_window = float(gcfg.get('accumulate_window', 3.0))
        self.dedupe_voxel = float(gcfg.get('dedupe_voxel', 0.02))
        # 场景一移动（物体被夹空碰移 / 被抓走）就清空累积候选池：否则旧位置的候选会在
        # accumulate_window 秒内继续被发布，下游会"对着已移走的旧位置反复下压"（实测症状）。
        self.pool_move_tol = float(gcfg.get('pool_move_tol', 0.015))
        self._prev_centroids = []
        self._pool = []
        min_w = float(cfg['grasp']['min_grasp_width'])
        max_w = float(cfg['grasp']['max_grasp_width'])
        # 宽度安全边距：所需开口必须比物理最大开口再小 margin，避免"刚好卡满"导致闭合失败
        self.width_margin = float(gcfg.get('width_safety_margin', 0.01))
        self.width_range = (min_w, max_w - self.width_margin)
        # 抓取中心与点云聚类质心的最大偏差：超出则说明指尖去怼物体边缘，直接丢弃
        self.cluster_center_tol = float(gcfg.get('cluster_center_tol', 0.03))
        # 高度是否夹进"抓取深度安全带"（见 _recenter_to_cluster）；关掉则完全用 GraspNet 高度
        self.recenter_z = bool(gcfg.get('recenter_z', True))
        # 抓取深度安全带：夹取点至少离顶面 min_depth（避免贴顶空抓），
        # 至多离顶面 max_depth（避免插入过深导致够不着/撞邻物）
        self.grasp_min_depth = float(gcfg.get('grasp_min_depth', 0.010))
        self.grasp_max_depth = float(gcfg.get('grasp_max_depth', 0.030))
        self._centroids = []
        self._centroid_extents = []
        self.min_points = int(gcfg.get('min_input_points', 500))
        self.infer_period = float(gcfg.get('infer_period', 1.0))

        ckpt = gcfg.get('checkpoint') or os.path.join(repo_root, 'checkpoint-rs.tar')
        net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                       cylinder_radius=0.05, hmin=-0.02,
                       hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        self.device = torch.device(gcfg.get('device', 'cuda:0'))
        net.to(self.device)
        ck = torch.load(ckpt, map_location=self.device)
        net.load_state_dict(ck['model_state_dict'])
        net.eval()
        self.net = net
        self.get_logger().info(
            'GraspNet 已加载 (epoch %s) | %s | 权重 %s'
            % (ck.get('epoch'), self.device, os.path.basename(ckpt)))

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            PointCloud2, gcfg.get('input_cloud', '/sorting/workspace_cloud'),
            self.on_cloud, qos)
        # 聚类质心来自预处理节点发布的 /sorting/clusters（CUBE marker 的位姿即质心），
        # 用于把"抓取中心离物体太远"的候选挡掉（防止指尖怼边缘）。
        clusters_topic = cfg.get('perception', {}).get('output_markers', '/sorting/clusters')
        self.create_subscription(MarkerArray, clusters_topic, self._on_clusters, 5)
        self.pub_grasps = self.create_publisher(
            Grasp6DArray, gcfg.get('output_grasps', '/sorting/graspnet_grasps'), 3)
        self.pub_markers = self.create_publisher(
            MarkerArray, gcfg.get('output_markers', '/sorting/graspnet_markers'), 3)

        self._last_infer = 0.0
        self.get_logger().info('GraspNet 抓取检测节点启动')

    # ------------------------------------------------------------------
    def _on_clusters(self, msg: MarkerArray):
        """缓存聚类质心与包围盒尺寸（CUBE marker：位姿=质心，scale=尺寸）。"""
        centroids = []
        extents = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE and marker.action == Marker.ADD:
                centroids.append((marker.pose.position.x,
                                  marker.pose.position.y,
                                  marker.pose.position.z))
                extents.append((marker.scale.x, marker.scale.y, marker.scale.z))
        if centroids:
            self._centroids = centroids
            self._centroid_extents = extents

    def _infer(self, pts_optical: np.ndarray):
        torch = self.torch
        n = len(pts_optical)
        rng = np.random.default_rng(0)
        idx = (rng.choice(n, self.num_point, replace=False) if n >= self.num_point
               else np.concatenate([np.arange(n),
                                    rng.choice(n, self.num_point - n, replace=True)]))
        cloud = torch.from_numpy(pts_optical[idx].astype(np.float32)[np.newaxis]).to(self.device)
        with torch.no_grad():
            preds = self.pred_decode(self.net({'point_clouds': cloud,
                                               'cloud_colors': np.zeros((self.num_point, 3),
                                                                       dtype=np.float32)}))
        return preds[0].detach().cpu().numpy()

    def _dedupe(self, pool):
        """按体素网格去重：同一格内只保留评分最高的抓取。"""
        v = max(self.dedupe_voxel, 1e-6)
        best = {}
        for item in pool:
            c = item[1]
            key = (int(np.floor(c[0] / v)), int(np.floor(c[1] / v)), int(np.floor(c[2] / v)))
            if key not in best or item[4] > best[key][4]:
                best[key] = item
        return list(best.values())

    def on_cloud(self, msg: PointCloud2):
        now = time.time()
        if (now - self._last_infer) < self.infer_period:
            return
        self._last_infer = now
        t0 = time.perf_counter()

        raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if isinstance(raw, np.ndarray) and raw.dtype.names:
            pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
        else:
            pts = np.array(list(raw), dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) < self.min_points:
            self.get_logger().warn('输入点云过少(%d)，跳过推理' % len(pts),
                                   throttle_duration_sec=5.0)
            return

        arr = self._infer(pts)
        if len(arr) == 0:
            self.get_logger().warn('GraspNet 未产生抓取', throttle_duration_sec=5.0)
            return

        score, width = arr[:, 0], arr[:, 1]
        center = arr[:, 13:16]
        rot = arr[:, 4:13].reshape(-1, 3, 3)
        n_raw = len(arr)

        # 光学系 -> world 系（先转系，因为接近方向过滤要在 world 系下判断"是否朝下"）
        world_center = center @ self.r_world_optical.T + self.t_world_cam
        world_rot = np.einsum('ij,njk->nik', self.r_world_optical, rot)

        # ---- 接近方向吸附到垂直 ----
        # 详见 _snap_approach_to_vertical：保留 GraspNet 判断的抓取中心与闭合轴，
        # 只把"从哪来"改成竖直向下。必须放在接近角过滤**之前**，否则大量侧向提议
        # 会被此前的角度门直接丢掉（实测 16° 门每帧只剩 1~5 条）。
        raw_tilt = np.degrees(np.arccos(np.clip(-world_rot[:, 2, 0], -1.0, 1.0)))
        world_rot, snap_valid = self._snap_approach_to_vertical(world_rot)
        n_snapped = int(snap_valid.sum())
        # 吸附后必须重定心：只改"从哪来"却保留侧面抓取中心，等于从物体边缘外侧下压
        world_center, recenter_mm = self._recenter_to_cluster(world_center)

        # ---- 三道硬过滤（逐级计数，防止过滤过狠导致零候选）----
        # 1) 评分 + 夹爪开口（上限已含 width_margin 安全边距）
        keep = snap_valid & (score >= self.score_threshold) & \
               (width >= self.width_range[0]) & (width <= self.width_range[1])
        n_score_width = int(keep.sum())

        # 2) 接近方向：强制接近垂直（规避斜抓碰撞与逆运动学无解）
        if self.max_approach_deg < 180.0:
            approach_z = world_rot[:, 2, 0]           # R[:,0] 为接近方向（graspnetAPI 约定）
            cos_limit = np.cos(np.deg2rad(self.max_approach_deg))
            keep &= (-approach_z) >= cos_limit
        n_approach = int(keep.sum())

        # 3) 抓取中心与最近聚类质心的偏差：过大说明指尖去怼物体边缘
        if self._centroids and self.cluster_center_tol > 0.0:
            centroids = np.asarray(self._centroids, dtype=float)
            idx = np.nonzero(keep)[0]
            if len(idx) > 0:
                diff = world_center[idx][:, None, :] - centroids[None, :, :]
                nearest = np.linalg.norm(diff, axis=2).min(axis=1)
                new_keep = np.zeros_like(keep)
                new_keep[idx] = nearest <= self.cluster_center_tol
                keep = new_keep
        n_centroid = int(keep.sum())

        score, width = score[keep], width[keep]
        world_center, world_rot = world_center[keep], world_rot[keep]
        if len(score) == 0:
            self.get_logger().warn(
                '过滤后无可用抓取 (原始 %d -> 评分/开口 %d -> 接近角≤%.0f° %d -> 质心≤%.0fmm %d)'
                % (n_raw, n_score_width, self.max_approach_deg, n_approach,
                   self.cluster_center_tol * 1000.0, n_centroid),
                throttle_duration_sec=5.0)
            return

        order = np.argsort(-score)[:self.max_grasps]
        score, width = score[order], width[order]
        world_center, world_rot = world_center[order], world_rot[order]

        # ---- 场景移动检测：任一聚类质心位移超过阈值即清空累积池 ----
        # 目的：物体被夹空碰移后，旧位置的候选必须立即失效，不能继续累积发布。
        if self._prev_centroids and self._centroids:
            cur = np.asarray(self._centroids, dtype=float)
            prev = np.asarray(self._prev_centroids, dtype=float)
            nearest = np.linalg.norm(
                cur[:, None, :] - prev[None, :, :], axis=2).min(axis=1)
            moved = float(nearest.max())
            if moved > self.pool_move_tol:
                self.get_logger().info(
                    f'[pool] 场景移动 {moved * 1000:.0f}mm > '
                    f'{self.pool_move_tol * 1000:.0f}mm，清空累积候选池')
                self._pool = []
        if self._centroids:
            self._prev_centroids = list(self._centroids)

        # ---- 时序累积 ----
        # 合成场景静止，而 GraspNet 单帧只对部分物体提出抓取（实测约 2/4）。
        # 累积最近若干帧的抓取并按体素去重，可让每个物体都拿到候选。
        now = time.time()
        self._pool = [(t, c, r, w, s) for (t, c, r, w, s) in self._pool
                      if (now - t) < self.accumulate_window]
        self._pool.extend((now, world_center[i].copy(), world_rot[i].copy(),
                           float(width[i]), float(score[i])) for i in range(len(score)))
        self._pool = self._dedupe(self._pool)
        pool_sorted = sorted(self._pool, key=lambda x: -x[4])[:self.max_grasps]
        world_center = np.array([p[1] for p in pool_sorted])
        world_rot = np.array([p[2] for p in pool_sorted])
        width = np.array([p[3] for p in pool_sorted])
        score = np.array([p[4] for p in pool_sorted])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'world'

        out = Grasp6DArray()
        out.header = header
        for i in range(len(score)):
            g = Grasp6D()
            g.header = header
            qx, qy, qz, qw = _matrix_to_quaternion(world_rot[i])
            g.pose = Pose()
            g.pose.position.x, g.pose.position.y, g.pose.position.z = \
                float(world_center[i, 0]), float(world_center[i, 1]), float(world_center[i, 2])
            g.pose.orientation.x, g.pose.orientation.y = float(qx), float(qy)
            g.pose.orientation.z, g.pose.orientation.w = float(qz), float(qw)
            ax = world_rot[i][:, 0]
            g.approach = Vector3(x=float(ax[0]), y=float(ax[1]), z=float(ax[2]))
            g.width = float(width[i])
            g.score = float(score[i])
            out.grasps.append(g)
        self.pub_grasps.publish(out)
        self._publish_markers(header, world_center, world_rot, score)

        dt = (time.perf_counter() - t0) * 1000.0
        # 逐帧诊断信息改为 debug：默认 INFO 级别不再刷屏（需要时用 --log-level debug 查看）
        self.get_logger().debug(
            '原始 %d（吸附 %d 条，平均倾角 %.0f°->0°，重定心均值 %.0fmm）-> 评分/开口 %d '
            '-> 接近角 %d -> 质心 %d -> 发布 Top-%d | 最高分 %.3f | %.0f ms'
            % (n_raw, n_snapped,
               float(raw_tilt[snap_valid].mean()) if n_snapped else 0.0,
               recenter_mm, n_score_width, n_approach, n_centroid,
               len(score), float(score[0]), dt),
            throttle_duration_sec=2.0)

    @staticmethod
    def _snap_approach_to_vertical(world_rot: np.ndarray):
        """把每条抓取的接近方向吸附到竖直向下，返回 (新旋转矩阵, 有效掩码)。

        为什么必须做：GraspNet 训练于侧面堆叠场景，对俯视相机会给出大量**侧向**接近
        的提议（实测 22° 门每帧几十条、16° 门每帧只剩 1~5 条）。而本项目的平行夹爪
        只能俯视下抓，且几何上存在硬约束：

            可抓深度 = 物体高 - (桌面安全余量 + 手指长·sinθ)

        手指长 53.7mm，对 30mm 高的 green_bar 要夹牢（深度 ≳10mm）必须 θ ≤ 16°。
        与其把这些提议丢弃，不如只改"从哪来"：抓取中心（指尖落点）与闭合轴
        （两指张开方向）都保留 GraspNet 的判断，接近方向改为竖直向下。

        R 的列按 graspnetAPI 约定为 [approach, closing, height]：
            approach' = (0, 0, -1)
            closing'  = 原 closing 在水平面内的分量（归一化）
            height'   = approach' × closing'（保证 R 仍是右手正交阵）
        原闭合轴几乎竖直（水平分量过小 = 两指张合方向无法定义）的提议判为无效。
        """
        n = len(world_rot)
        if n == 0:
            return world_rot, np.zeros(0, dtype=bool)

        down = np.array([0.0, 0.0, -1.0])
        closing = world_rot[:, :, 1]
        # 去掉竖直分量：闭合轴必须落在水平面内（两指只能水平张合）
        horiz = np.column_stack([closing[:, 0], closing[:, 1], np.zeros(n)])
        norm = np.linalg.norm(horiz, axis=1)
        valid = norm > 1e-3

        c_new = horiz / np.where(valid, norm, 1.0)[:, None]
        h_new = np.cross(np.broadcast_to(down, c_new.shape), c_new)

        new_rot = world_rot.copy()
        new_rot[:, :, 0] = down
        new_rot[:, :, 1] = c_new
        new_rot[:, :, 2] = h_new
        return new_rot, valid

    def _recenter_to_cluster(self, centers: np.ndarray):
        """把吸附后的抓取中心水平对齐到最近聚类质心，返回 (新中心, 平均修正量mm)。

        为什么必须做：GraspNet 对俯视相机给出的提议大多是**侧向**抓取，抓取中心落在
        物体的**侧面**上 —— 偏离物体中线约半个物宽。实测候选对簇质心的水平偏移中位数
        达 35mm，而物体本身只有 30~50mm 宽，即中心根本不在物体上。此时只把接近方向
        吸附成竖直、却保留这个侧面中心，夹爪就是"从物体边缘外侧垂直下压"：一根手指
        必然压在物体本体上（实测接触深度中位 4.8mm、39% 超过 8mm，下压被规划层拒绝
        占全部失败的 74%）。

        修正把水平面内两个方向都对齐到簇质心，因此：
          * 闭合方向上落到物体中线 —— 两指对称跨夹，不再压本体；
          * 沿物体长度方向也回到质心 —— 避免 GraspNet 的端部/边缘落点导致夹边滑落。
        竖直高度**不强行拉到中线**（那样对矮物体要"插入过深"、会够不着或撞邻物），
        而是夹进"抓取深度安全带"：至少离顶面 min_depth、至多离顶面 max_depth。
        """
        if not self._centroids or len(centers) == 0:
            return centers, 0.0

        centroids = np.asarray(self._centroids, dtype=float)
        out = centers.copy()
        shifts = []
        for i in range(len(centers)):
            # 找水平面内最近的簇质心（只用 xy，避免高度差把邻居选进来）
            diff_xy = centroids[:, :2] - centers[i, :2]
            nearest = int(np.argmin(np.linalg.norm(diff_xy, axis=1)))
            step = float(np.hypot(centers[i, 0] - centroids[nearest][0],
                                  centers[i, 1] - centroids[nearest][1]))
            out[i, 0] = centroids[nearest][0]
            out[i, 1] = centroids[nearest][1]
            # 高度也回到簇质心（物体竖直中线）：GraspNet 常把抓取中心提议在物体
            # **上边缘**附近，指尖随之贴顶下压 -> 实测会夹空并把物体顶飞。回到中线后
            # 两指在物体中部对称跨夹，既夹得稳、又不会顶到顶面把物体撞跑。
            if self.recenter_z and nearest < len(self._centroid_extents):
                # 不强行拉到竖直中线（那样"插入过深"）：改为把高度夹进安全带。
                # 上界 = 顶面 - min_depth：保证至少夹住 min_depth 深，避免贴顶空抓；
                # 下界 = 顶面 - max_depth：限制最大插入深度，避免插太深够不着/撞邻物。
                centroid_z = float(centroids[nearest][2])
                height = float(self._centroid_extents[nearest][2])
                top = centroid_z + 0.5 * height
                z_hi = top - self.grasp_min_depth
                z_lo = top - min(self.grasp_max_depth,
                                 max(0.5 * height, self.grasp_min_depth))
                out[i, 2] = float(min(max(float(centers[i, 2]), z_lo), z_hi))
            shifts.append(step)
        return out, (float(np.mean(shifts)) * 1000.0 if shifts else 0.0)

    def _publish_markers(self, header, centers, rots, scores):
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (c, r, s) in enumerate(zip(centers, rots, scores)):
            # 抓取中心
            m = Marker()
            m.header = header
            m.ns = 'grasp_center'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                float(c[0]), float(c[1]), float(c[2])
            m.pose.orientation.w = 1.0
            r_size = 0.012 if i == 0 else 0.007
            m.scale.x = m.scale.y = m.scale.z = r_size
            # 最高分红色，其余绿色
            m.color.r, m.color.g, m.color.b, m.color.a = \
                (0.95, 0.2, 0.2, 1.0) if i == 0 else (0.2, 0.85, 0.4, 0.9)
            arr.markers.append(m)

            # 接近方向箭头（长度 30mm）
            a = Marker()
            a.header = header
            a.ns = 'grasp_approach'
            a.id = i
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.position.x, a.pose.position.y, a.pose.position.z = \
                float(c[0]), float(c[1]), float(c[2])
            qx, qy, qz, qw = _matrix_to_quaternion(r)
            a.pose.orientation.x, a.pose.orientation.y = float(qx), float(qy)
            a.pose.orientation.z, a.pose.orientation.w = float(qz), float(qw)
            a.scale.x, a.scale.y = 0.03, 0.005
            a.color.r, a.color.g, a.color.b, a.color.a = 0.2, 0.5, 0.95, 0.9
            arr.markers.append(a)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()