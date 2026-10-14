"""模块2.5 点云预处理：把相机原始点云处理成"可直接喂给抓取网络"的工作区点云。

处理链（Gate1–3 的结论直接驱动，见 README 的 Gate 记录）：
    1. 坐标系统一：Gazebo 原始点云是 body 系(x前/y左/z上)，本节点统一转到
       相机光学系(z前/x右/y下) 与 world 系，消除下游几何错位
    2. 工作区 ROI 裁剪 + 高度夹层：剔除桌面下方、以及高处机械臂自遮挡点
    3. RANSAC 平面分割：剔除桌面平面（合成场景中桌面占 ~99% 的点，
       不剔除则抓取网络采样不到物体）
    4. 离群点滤波 + DBSCAN 欧式聚类：把点云切成一个个物体簇
    4b. 用 YOLO 类别标签图把 DBSCAN 因 1.5mm 紧邻而误合并的"多物体簇"按类别拆成
        多个子簇（本项目约束"每类各一个"，故类别即实例）
    5. 发布：工作区点云(供抓取网络) + 聚类可视化 MarkerArray(供 rviz)

算法实现复用成熟开源库：
    * scikit-learn  —— DBSCAN 聚类 / RANSAC 平面拟合（工业界广泛使用）
    * numpy         —— 向量化几何运算
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from sklearn.cluster import DBSCAN
from sklearn.base import BaseEstimator
from sklearn.linear_model import RANSACRegressor

from sorting_bringup import R_BODY_TO_OPTICAL, camera_extrinsics, load_system_config


class _Plane(BaseEstimator):
    """平面拟合器 z = a*x + b*y + c，供 sklearn RANSACRegressor 使用。"""

    def fit(self, X, y):
        A = np.column_stack([X, np.ones(len(X))])
        self.coef_, *_ = np.linalg.lstsq(A, y, rcond=None)
        return self

    def predict(self, X):
        A = np.column_stack([X, np.ones(len(X))])
        return A @ self.coef_

    def score(self, X, y):
        """sklearn RANSACRegressor 要求估计器实现 score（用 R²）。"""
        pred = self.predict(X)
        ss_res = float(((y - pred) ** 2).sum())
        ss_tot = float(((y - y.mean()) ** 2).sum())
        return 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 0.0


class PointCloudPreprocessor(Node):
    """点云预处理节点。"""

    def __init__(self):
        super().__init__('pointcloud_preprocessor')

        cfg = load_system_config()
        self.cfg = cfg
        topics = cfg['topics']
        scene = cfg['scene']
        self.pcfg = cfg.get('perception', {})

        # ---- 相机外参（唯一真值源见 sorting_bringup.camera；不依赖 TF，确定性更强）----
        # 注意：必须用完整 RPY，此前只用 pitch（Ry(pitch)）只在"光轴朝正下方"时成立。
        self.t_world_cam, self.r_world_optical = camera_extrinsics(cfg)
        self.r_optical_from_body = R_BODY_TO_OPTICAL.T  # 正交阵，转置即逆

        # ---- ROI / 高度夹层 ----
        roi = scene['workspace_roi']
        self.roi = (roi['x_min'], roi['x_max'], roi['y_min'], roi['y_max'])
        self.table_z = float(scene['table']['top_z'])
        self.z_min = self.table_z + float(self.pcfg.get('z_below_table', -0.03))
        self.z_max = self.table_z + float(self.pcfg.get('z_above_table', 0.25))

        # ---- 算法参数 ----
        self.plane_thresh = float(self.pcfg.get('plane_distance_threshold', 0.008))
        self.plane_trials = int(self.pcfg.get('plane_max_trials', 120))
        self.plane_fit_points = int(self.pcfg.get('plane_fit_points', 20000))
        self.cluster_eps = float(self.pcfg.get('cluster_eps', 0.018))
        self.cluster_min_points = int(self.pcfg.get('cluster_min_points', 30))
        self.min_cluster_points = int(self.pcfg.get('min_cluster_points', 120))
        self.max_cluster_points = int(self.pcfg.get('max_cluster_points', 200000))
        self.max_cluster_extent = float(self.pcfg.get('max_cluster_extent', 0.30))
        self.outlier_neighbors = int(self.pcfg.get('outlier_neighbors', 20))
        self.outlier_std_ratio = float(self.pcfg.get('outlier_std_ratio', 2.0))

        self.input_topic = topics['point_cloud']
        self.output_cloud_topic = self.pcfg.get('output_cloud', '/sorting/workspace_cloud')
        self.output_markers_topic = self.pcfg.get('output_markers', '/sorting/clusters')

        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud, qos
        )

        # ---- YOLO 类别标签图：把 DBSCAN 合并的"紧邻多物体簇"按类别拆开 ----
        # 背景：物体边界间隙仅 1.5mm，DBSCAN 会把 4 个物体连成一个簇，导致下游把整团
        # 当作单个物体（候选塌缩、两次失败即整团拉黑）。YOLO 分割在 2D 上能分开它们，
        # 这里把簇内每个 3D 点投影到像素、读类别标签，按类别分组 -> 一类别一组即一物体。
        self.split_by_label = bool(self.pcfg.get('split_by_label', True))
        self.split_min_points = int(self.pcfg.get('split_min_points', 30))
        ycfg = cfg.get('yolo_seg', {})
        self.label_topic = ycfg.get('output_label', '/sorting/class_label_image')
        self._label_img = None
        if self.split_by_label:
            self.create_subscription(Image, self.label_topic, self._on_label, qos)
        # 相机内参：由配置的 FOV/分辨率推出（与仿真相机一致），供 3D 点 -> 像素投影。
        # 与 _to_world/_world_to_optical 用同一套外参，正反变换误差相互抵消，
        # 因此投影与 RGB/标签图严格对齐。
        cam = cfg['camera']
        w_cam, h_cam = int(cam['width']), int(cam['height'])
        self.fx = (w_cam / 2.0) / np.tan(float(cam['horizontal_fov']) / 2.0)
        self.fy = self.fx
        self.cx = (w_cam - 1) / 2.0
        self.cy = (h_cam - 1) / 2.0
        self.pub_cloud = self.create_publisher(PointCloud2, self.output_cloud_topic, 3)
        self.pub_markers = self.create_publisher(MarkerArray, self.output_markers_topic, 3)

        self.frames = cfg['frames']
        self._last_log = 0.0
        self.get_logger().info(
            '点云预处理节点启动: %s -> %s (ROI x[%.2f,%.2f] y[%.2f,%.2f] z[%.3f,%.3f])'
            % (self.input_topic, self.output_cloud_topic, *self.roi, self.z_min, self.z_max)
        )

    # ------------------------------------------------------------------
    def _to_world(self, pts: np.ndarray) -> np.ndarray:
        """body 系原始点 -> world 系。

        两步：body -> optical（相机装的是光学系定义），再 optical -> world。
        """
        optical = pts @ self.r_optical_from_body.T
        return optical @ self.r_world_optical.T + self.t_world_cam

    def _world_to_optical(self, pts: np.ndarray) -> np.ndarray:
        """world 系 -> 相机光学系（抓取网络按光学系训练，必须喂光学系点云）。"""
        return (pts - self.t_world_cam) @ self.r_world_optical

    def _fit_plane(self, pts: np.ndarray):
        """RANSAC 拟合桌面平面，返回 (coef, 全部点的平面残差)。"""
        n = len(pts)
        idx = (np.random.choice(n, self.plane_fit_points, replace=False)
               if n > self.plane_fit_points else np.arange(n))
        est = RANSACRegressor(
            estimator=_Plane(),
            min_samples=3,          # 平面拟合最少 3 点
            residual_threshold=self.plane_thresh,
            max_trials=self.plane_trials,
            random_state=0,
        )
        est.fit(pts[idx, :2], pts[idx, 2])
        plane = est.estimator_
        resid = pts[:, 2] - plane.predict(pts[:, :2])
        return plane, resid

    def _publish_cloud(self, pts: np.ndarray, header: Header):
        msg = point_cloud2.create_cloud_xyz32(header, pts.astype(np.float32))
        self.pub_cloud.publish(msg)

    def _publish_markers(self, header: Header, clusters):
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (centroid, extent, npts, sub) in enumerate(clusters):
            color = np.array([
                (0.9, 0.25, 0.25), (0.25, 0.6, 0.95), (0.25, 0.85, 0.35),
                (0.95, 0.8, 0.2), (0.75, 0.35, 0.9), (0.3, 0.85, 0.85),
            ][i % 6])

            box = Marker()
            box.header = header
            box.ns = 'cluster_box'
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = centroid
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = np.maximum(extent, 0.006)
            box.color.r, box.color.g, box.color.b, box.color.a = *color, 0.45
            arr.markers.append(box)

            pts_marker = Marker()
            pts_marker.header = header
            pts_marker.ns = 'cluster_points'
            pts_marker.id = i
            pts_marker.type = Marker.POINTS
            pts_marker.action = Marker.ADD
            pts_marker.scale.x = pts_marker.scale.y = 0.004
            pts_marker.color.r, pts_marker.color.g, pts_marker.color.b, pts_marker.color.a = *color, 1.0
            sample = sub[::max(1, len(sub) // 1500)]
            pts_marker.points = [
                Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in sample
            ]
            arr.markers.append(pts_marker)

        self.pub_markers.publish(arr)

    def _on_label(self, msg: Image):
        """缓存 YOLO 输出的逐像素类别标签图（mono8：0=背景，class_id+1）。"""
        if msg.encoding not in ('mono8', '8UC1'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size != msg.height * msg.width:
            return
        self._label_img = arr.reshape(msg.height, msg.width)

    def _split_cluster_by_label(self, pts: np.ndarray):
        """用类别标签图把一个簇拆成多个子簇（返回点数组列表）。

        本项目约束"每类各一个"，故**类别即实例**：按投影像素的类别分组，每组即一个物体。
        未被任何掩码覆盖的点（标签 0）及其它过小的类别碎片合并为兜底组，避免 YOLO 漏检
        时把物体整块丢掉；若拆分结果为空则退回原始整簇。
        """
        label_img = self._label_img
        if not self.split_by_label or label_img is None or len(pts) == 0:
            return [pts]

        optical = self._world_to_optical(pts)
        z = optical[:, 2]
        valid = z > 1e-3
        u = np.zeros(len(pts), dtype=int)
        v = np.zeros(len(pts), dtype=int)
        u[valid] = np.round(self.fx * optical[valid, 0] / z[valid] + self.cx).astype(int)
        v[valid] = np.round(self.fy * optical[valid, 1] / z[valid] + self.cy).astype(int)
        h, w = label_img.shape[:2]
        inside = valid & (u >= 0) & (u < w) & (v >= 0) & (v < h)

        lab = np.zeros(len(pts), dtype=np.int32)
        lab[inside] = label_img[v[inside], u[inside]]

        groups = []
        residual = np.zeros(len(pts), dtype=bool)
        for value in np.unique(lab):
            sel = (lab == value)
            if value != 0 and int(sel.sum()) >= self.split_min_points:
                groups.append(pts[sel])
            else:
                residual |= sel
        # 兜底组只在"完全没有类别命中"（YOLO 漏检）时才启用；否则残余点多为掩码
        # 边缘未被覆盖的碎片，会形成一个横跨整团的多余质心、进而把抓取中心拉偏。
        if not groups and int(residual.sum()) >= self.min_cluster_points:
            groups.append(pts[residual])
        return groups if groups else [pts]

    # ------------------------------------------------------------------
    def on_cloud(self, msg: PointCloud2):
        import time
        t0 = time.perf_counter()

        raw = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        # read_points 在新版返回结构化 ndarray；直接取列，避免 list() 逐个装箱（性能关键）
        if isinstance(raw, np.ndarray) and raw.dtype.names:
            pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
        else:
            arr = np.array(list(raw))
            if arr.dtype.names:
                pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
            else:
                pts = arr.astype(np.float64)

        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if len(pts) < 100:
            self.get_logger().warn('有效点过少: %d' % len(pts), throttle_duration_sec=5.0)
            return

        world = self._to_world(pts)

        # ---- ROI + 高度夹层（剔除桌面下方与高处机械臂）----
        m = ((world[:, 0] > self.roi[0]) & (world[:, 0] < self.roi[1])
             & (world[:, 1] > self.roi[2]) & (world[:, 1] < self.roi[3])
             & (world[:, 2] > self.z_min) & (world[:, 2] < self.z_max))
        w = world[m]
        n_roi = len(w)
        if n_roi < 100:
            self.get_logger().warn('ROI 内点过少: %d' % n_roi, throttle_duration_sec=5.0)
            return
        self._to_world_pts = w

        # ---- RANSAC 桌面平面剔除 ----
        _, resid = self._fit_plane(w)
        keep = np.abs(resid) > self.plane_thresh
        obj = w[keep]
        n_obj = len(obj)
        if n_obj < 100:
            self.get_logger().warn('平面剔除后点过少: %d' % n_obj, throttle_duration_sec=5.0)
            return

        alpha = self.pcfg.get('downsample_alpha', 1.0)
        work = obj[::max(1, int(1.0 / max(alpha, 1e-6)))] if alpha < 1.0 else obj

        # ---- DBSCAN 欧式聚类 ----
        labels = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_points,
                        n_jobs=-1).fit_predict(work)
        clusters = []
        for lab in sorted(set(labels.tolist())):
            if lab < 0:
                continue
            sub = work[labels == lab]
            if not (self.min_cluster_points <= len(sub) <= self.max_cluster_points):
                continue
            # 用 YOLO 类别标签把"紧邻多物体"合并簇拆成多个子簇（每类即一个物体）
            for sub_pts in self._split_cluster_by_label(sub):
                lo, hi = sub_pts.min(axis=0), sub_pts.max(axis=0)
                extent = hi - lo
                if np.max(extent) > self.max_cluster_extent:
                    continue
                clusters.append(((lo + hi) / 2.0, extent, len(sub_pts), sub_pts))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frames['world']

        self._publish_markers(header, clusters)

        # 点云按抓取网络的训练约定发布为"相机光学系"（z 前 / x 右 / y 下）
        cloud_header = Header()
        cloud_header.stamp = msg.header.stamp
        cloud_header.frame_id = self.frames['camera_optical']
        # 只发布"通过筛选的物体簇"点云，而不是去平面后的全部点：
        # 后者仍含料箱壁残片/桌面边缘等杂点，抓取网络会在这些杂点上提出无效抓取。
        if clusters:
            obj_pts = np.concatenate([c[3] for c in clusters])
        else:
            obj_pts = work[:0]
        self._publish_cloud(self._world_to_optical(obj_pts), cloud_header)

        dt = (time.perf_counter() - t0) * 1000.0
        if (time.time() - self._last_log) > 1.0:
            self._last_log = time.time()
            self.get_logger().info(
                '原始 %d -> ROI %d -> 去桌面 %d -> 聚类 %d 个 | %.1f ms'
                % (len(pts), n_roi, n_obj, len(clusters), dt)
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()