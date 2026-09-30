"""仿真合成数据集自动生成。

流程（teach.txt 的"仿真自动生成 + 域随机化"数据策略）：
  1. 调用 /sorting/reset_scene 随机重摆场景（物体位姿/类别/颜色/光照/相机抖动）
  2. 等待物理稳定，取一帧 RGB 图像
  3. 读取 /sorting/ground_truth 真值 + TF(world -> camera_optical_frame) 外参
  4. 将每个物体的三维几何投影到图像平面，得到轮廓多边形
  5. 按深度排序做可见性光栅化，剔除被遮挡部分
  6. 输出 YOLO-seg 格式标注（归一化多边形）+ data.yaml

由于标注由几何投影生成，标签与图像严格对齐，可批量产出上万张样本。
"""

import os
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
import tf_transformations
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import ObjectStateArray
from sorting_interfaces.srv import ResetScene


class SyntheticDatasetGenerator(Node):
    """合成分割数据集生成节点。"""

    def __init__(self):
        super().__init__('synthetic_dataset_generator')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        topics = self.system_config['topics']
        self.topics = topics

        self.declare_parameter('output_dir', 'datasets/sorting_seg')
        self.declare_parameter('num_samples', 300)
        self.declare_parameter('val_ratio', 0.2)
        self.declare_parameter('seed', 20260930)
        self.declare_parameter('min_visibility', 0.25)
        self.declare_parameter('save_preview', True)

        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.val_ratio = float(self.get_parameter('val_ratio').value)
        self.min_visibility = float(self.get_parameter('min_visibility').value)
        self.save_preview = bool(self.get_parameter('save_preview').value)

        np.random.seed(int(self.get_parameter('seed').value))

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topics['image_raw'], self._on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, topics['camera_info'], self._on_camera_info, sensor_qos
        )
        self.create_subscription(
            ObjectStateArray, topics['ground_truth'], self._on_ground_truth, 10
        )

        self.reset_client = self.create_client(ResetScene, self.system_config['services']['reset_scene'])

        self._image: Optional[np.ndarray] = None
        self._image_seq = 0
        self._camera_info: Optional[CameraInfo] = None
        self._ground_truth: Optional[ObjectStateArray] = None

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            self._image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._image_seq += 1
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败: {exc}', throttle_duration_sec=5.0)

    def _on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _on_ground_truth(self, msg: ObjectStateArray):
        self._ground_truth = msg

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _spin_until(self, predicate, timeout: float, period: float = 0.05) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=period)
            if predicate():
                return True
        return predicate()

    def _wait_inputs(self, timeout: float = 20.0) -> bool:
        return self._spin_until(
            lambda: self._image is not None
            and self._camera_info is not None
            and self._ground_truth is not None,
            timeout,
        )

    def _call_reset_scene(self) -> bool:
        if not self.reset_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('reset_scene 服务不可用')
            return False
        request = ResetScene.Request()
        request.n_objects = int(self.system_config['scene'].get('n_objects', 4))
        request.randomize_class = True
        request.randomize_pose = True
        request.randomize_scene = True
        future = self.reset_client.call_async(request)
        self._spin_until(lambda: future.done(), timeout=60.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('reset_scene 调用失败')
            return False
        return bool(future.result().success)

    def _lookup_world_to_camera(self) -> Optional[np.ndarray]:
        """返回 4x4 外参矩阵 T_camera_world。"""
        target = self.system_config['frames']['camera_optical']
        source = self.system_config['frames']['world']
        try:
            transform = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'TF 查询失败 ({target} <- {source}): {exc}', throttle_duration_sec=5.0)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = tf_transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        return matrix

    # ------------------------------------------------------------------
    # 几何投影
    # ------------------------------------------------------------------
    @staticmethod
    def _object_points(object_state) -> np.ndarray:
        """按物体形状生成世界坐标系下的采样点云（用于投影求轮廓）。"""
        size = np.array([object_state.size.x, object_state.size.y, object_state.size.z], dtype=float)
        pose = object_state.pose
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation = tf_transformations.quaternion_matrix(quat)[:3, :3]
        center = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

        shape = int(object_state.shape)
        if shape == 1:  # cylinder：上下两个圆
            radius = size[0] / 2.0
            angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
            circle = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
            local = []
            for dz in (-size[2] / 2.0, size[2] / 2.0):
                ring = np.concatenate([circle, np.full((circle.shape[0], 1), dz)], axis=1)
                local.append(ring)
            local = np.concatenate(local, axis=0)
        elif shape == 2:  # sphere：正对相机的圆盘近似
            radius = size[0] / 2.0
            angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
            local = np.stack(
                [radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles)], axis=1
            )
        else:  # box：八个角点
            half = size / 2.0
            local = np.array(
                [
                    [sx * half[0], sy * half[1], sz * half[2]]
                    for sx in (-1, 1)
                    for sy in (-1, 1)
                    for sz in (-1, 1)
                ]
            )

        world = (rotation @ local.T).T + center
        return world

    @staticmethod
    def _project(points_world: np.ndarray, extrinsic: np.ndarray, camera_info) -> np.ndarray:
        """世界点 -> 像素坐标。返回 (N, 3): [u, v, z]。"""
        homogeneous = np.concatenate([points_world, np.ones((points_world.shape[0], 1))], axis=1)
        camera_points = (extrinsic @ homogeneous.T).T[:, :3]

        z = camera_points[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = camera_info.k[0] * camera_points[:, 0] / z + camera_info.k[2]
            v = camera_info.k[4] * camera_points[:, 1] / z + camera_info.k[5]
        return np.stack([u, v, z], axis=1)

    # ------------------------------------------------------------------
    # 单帧标注生成
    # ------------------------------------------------------------------
    def _build_labels(self, image: np.ndarray, extrinsic: np.ndarray, camera_info):
        """返回 (lines, preview, stats)。lines 为 YOLO-seg 标注文本行。"""
        height, width = image.shape[:2]
        ground_truth = self._ground_truth

        entries = []
        for object_state in ground_truth.objects:
            points_world = self._object_points(object_state)
            projected = self._project(points_world, extrinsic, camera_info)
            if np.any(projected[:, 2] <= 0.05):
                continue  # 有点在相机后方

            uv = projected[:, :2]
            if not np.all(np.isfinite(uv)):
                continue

            hull = cv2.convexHull(uv.astype(np.float32)).reshape(-1, 2)
            if hull.shape[0] < 3:
                continue

            depth = float(np.mean(projected[:, 2]))
            entries.append(
                {
                    'class_id': int(object_state.class_id),
                    'class_name': object_state.class_name,
                    'hull': hull,
                    'depth': depth,
                    'center_z': float(object_state.pose.position.z),
                }
            )

        # 按深度从远到近光栅化 -> 近处物体覆盖远处物体，实现遮挡剔除
        id_map = np.zeros((height, width), dtype=np.int16)
        entries.sort(key=lambda item: -item['depth'])
        for index, entry in enumerate(entries):
            polygon = np.round(entry['hull']).astype(np.int32)
            cv2.fillPoly(id_map, [polygon], np.int16(index + 1))

        lines: List[str] = []
        preview = image.copy()
        stats = {'objects': len(entries), 'labeled': 0, 'occluded': 0}

        for index, entry in enumerate(entries):
            visible = (id_map == (index + 1)).astype(np.uint8)
            visible_area = int(visible.sum())

            full_polygon = np.round(entry['hull']).astype(np.int32)
            full_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(full_mask, [full_polygon], 1)
            full_area = max(int(full_mask.sum()), 1)
            visibility = visible_area / float(full_area)

            if visibility < self.min_visibility or visible_area < 40:
                stats['occluded'] += 1
                continue

            contours, _ = cv2.findContours(visible, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            largest = cv2.contourArea(contours[0])
            if largest < 30:
                continue

            polygons = []
            for contour in contours:
                if cv2.contourArea(contour) < max(0.15 * largest, 25):
                    continue
                epsilon = 0.004 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
                if approx.shape[0] < 3:
                    continue
                normalized = approx.astype(float) / np.array([width, height], dtype=float)
                polygons.append(np.clip(normalized, 0.0, 1.0))

            if not polygons:
                continue

            parts = []
            for polygon in polygons:
                flat = ' '.join(f'{value:.6f}' for value in polygon.reshape(-1))
                parts.append(flat)
            lines.append(f"{entry['class_id']} " + ' '.join(parts))
            stats['labeled'] += 1

            color = (
                int(entry['class_id'] * 60 % 255),
                int(200 - entry['class_id'] * 40 % 200),
                int(120 + entry['class_id'] * 30 % 120),
            )
            for polygon in polygons:
                pts = (polygon * np.array([width, height])).astype(np.int32)
                cv2.polylines(preview, [pts], True, color, 2)
            center = np.mean(np.vstack(polygons) * np.array([width, height]), axis=0)
            cv2.putText(
                preview, entry['class_name'],
                (int(center[0]), int(center[1])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
            )

        return lines, preview, stats

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def run(self) -> int:
        train_images = os.path.join(self.output_dir, 'images', 'train')
        train_labels = os.path.join(self.output_dir, 'labels', 'train')
        val_images = os.path.join(self.output_dir, 'images', 'val')
        val_labels = os.path.join(self.output_dir, 'labels', 'val')
        preview_dir = os.path.join(self.output_dir, 'preview')
        for path in (train_images, train_labels, val_images, val_labels, preview_dir):
            os.makedirs(path, exist_ok=True)

        self.get_logger().info(f'等待仿真输入（图像 / 相机内参 / 真值）...')
        if not self._wait_inputs(timeout=60.0):
            self.get_logger().error('未能获取仿真输入，请确认仿真与 scene_manager 已启动')
            return 1

        saved = 0
        attempt = 0
        max_attempts = self.num_samples * 4

        while saved < self.num_samples and attempt < max_attempts and rclpy.ok():
            attempt += 1
            if not self._call_reset_scene():
                self.get_logger().warn('重置场景失败，跳过本次采样')
                continue

            settle = float(self.system_config['task'].get('settle_time', 1.5))
            settle = max(settle, 1.0)
            # 等待物理稳定 + 新一帧图像
            seq_before = self._image_seq
            self._spin_until(lambda: self._image_seq > seq_before, timeout=settle + 5.0)

            image = self._image
            camera_info = self._camera_info
            if image is None or camera_info is None:
                continue

            extrinsic = self._lookup_world_to_camera()
            if extrinsic is None:
                continue

            lines, preview, stats = self._build_labels(image, extrinsic, camera_info)
            if stats['labeled'] == 0:
                self.get_logger().warn('本帧没有可标注目标，跳过', throttle_duration_sec=2.0)
                continue

            is_val = np.random.rand() < self.val_ratio
            split = 'val' if is_val else 'train'
            name = f'frame_{saved:05d}'
            image_dir = val_images if is_val else train_images
            label_dir = val_labels if is_val else train_labels

            cv2.imwrite(os.path.join(image_dir, name + '.jpg'), image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            with open(os.path.join(label_dir, name + '.txt'), 'w', encoding='utf-8') as handle:
                handle.write('\n'.join(lines) + '\n')

            if self.save_preview and saved % 10 == 0:
                cv2.imwrite(os.path.join(preview_dir, name + '_preview.jpg'), preview)

            saved += 1
            if saved % 20 == 0 or saved == self.num_samples:
                self.get_logger().info(
                    f'已生成 {saved}/{self.num_samples} 帧 '
                    f'(本帧目标 {stats["labeled"]}, 遮挡剔除 {stats["occluded"]})'
                )

        self._write_dataset_yaml()
        self.get_logger().info(f'数据集生成完成: {self.output_dir}（共 {saved} 帧）')
        return 0

    def _write_dataset_yaml(self):
        names = self.class_table.class_names
        content = [
            f'path: {os.path.abspath(self.output_dir)}',
            'train: images/train',
            'val: images/val',
            '',
            'names:',
        ]
        for index, name in enumerate(names):
            content.append(f'  {index}: {name}')
        with open(os.path.join(self.output_dir, 'data.yaml'), 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(content) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDatasetGenerator()
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()