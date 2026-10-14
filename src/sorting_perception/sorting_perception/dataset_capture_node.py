"""YOLOv8-seg 合成数据集采集与自动标注节点。

用途：
    在 Gazebo 仿真中自动采集相机 RGB 图，并利用仿真真值（/sorting/ground_truth）
    把每个物体投影到图像平面，生成 YOLOv8 实例分割标签（多边形）。

为什么能用自动标注：
    仿真提供两样真值，二者结合即可精确算出物体在像素上的轮廓——
      1. 每个物体的世界位姿与尺寸（ObjectState.pose / size / shape / class_id）；
      2. 相机真实外参（scene_manager 广播的 world -> camera_optical_frame TF，
         含域随机化后的相机抖动，比静态配置更准）+ 内参（/camera/camera_info）。

标注流程（每帧）：
    1. 按物体形状采样其表面轮廓点（box 取 8 角点；cylinder 取上下两圈边缘；
       sphere 取球面采样点），转到相机光学系后投影到像素；
    2. 对每个物体的投影点取凸包，得到该物体的"轮廓多边形"；
    3. **遮挡处理**：物体按到相机的距离从近到远依次栅格化，近者的像素优先占用
       （近处物体遮挡远处物体），保证标签反映"真实可见区域"而非几何投影；
    4. 对每个物体的可见像素提取外轮廓，约简后按 YOLOv8-seg 规范写出
       ``class_id x1 y1 x2 y2 ...``（坐标按图像宽高归一化）。

多样性（避免模型只记住一帧）：
    每次采集前调用 /sorting/reset_scene，令
      * randomize_scene=True  -> 相机安装位姿与光照随机化（scene_manager 支持）；
      * randomize_pose=True   -> "四种物体每种各一个"且**类别到槽位的排列随机**
                                 （见 scene_manager._spawn_objects 的 randomize_layout）。
    这样每帧的物体位置、朝向视角、光照都不同；而网格几何与 1.5mm 边界间隙保持不变。

运行（需 venv 依赖，见 scripts/capture_dataset.sh）：
    ros2 run sorting_perception dataset_capture --ros-args -p use_sim_time:=true
"""

import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import ObjectStateArray
from sorting_interfaces.srv import ResetScene

# 物体形状枚举（与 ObjectState.msg 一致）
SHAPE_BOX, SHAPE_CYLINDER, SHAPE_SPHERE = 0, 1, 2


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 -> 3x3 旋转矩阵。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


def _surface_points(shape: int, size: np.ndarray, n_rim: int = 48,
                    n_sphere: int = 120) -> np.ndarray:
    """在物体**本地坐标系**采样能刻画其轮廓的表面点（Nx3）。"""
    sx, sy, sz = size
    if shape == SHAPE_BOX:
        # 长方体：8 个角点；凸包投影即为其轮廓
        half = np.array([sx, sy, sz]) / 2.0
        signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
        return signs * half
    if shape == SHAPE_CYLINDER:
        # 圆柱：上下两个圆环边缘（本地 z 为轴）
        r = sx / 2.0
        ang = np.linspace(0.0, 2.0 * math.pi, n_rim, endpoint=False)
        ring = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros(n_rim)], axis=1)
        top = ring.copy()
        top[:, 2] = sz / 2.0
        bot = ring.copy()
        bot[:, 2] = -sz / 2.0
        return np.vstack([top, bot])
    # 球：球面均匀采样（Fibonacci 球）
    r = sx / 2.0
    i = np.arange(n_sphere) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n_sphere)
    theta = math.pi * (1.0 + 5.0 ** 0.5) * i
    return r * np.stack([np.cos(theta) * np.sin(phi),
                         np.sin(theta) * np.sin(phi),
                         np.cos(phi)], axis=1)


class DatasetCaptureNode(Node):
    """采集 RGB + 自动生成 YOLOv8-seg 标签。"""

    def __init__(self):
        super().__init__('dataset_capture')

        cfg = load_system_config()
        topics = cfg['topics']
        dcfg = cfg.get('dataset', {})
        self.class_table = ClassTable()

        out_dir = self.declare_parameter(
            'output_dir', dcfg.get('output_dir', 'datasets/yolo_seg')
        ).value
        num_images = int(self.declare_parameter(
            'num_images', int(dcfg.get('num_images', 200))
        ).value)
        val_ratio = float(self.declare_parameter(
            'val_ratio', float(dcfg.get('val_ratio', 0.15))
        ).value)
        self.settle_time = float(self.declare_parameter(
            'settle_time', float(dcfg.get('settle_time', 2.0))
        ).value)
        self.max_objects = int(self.declare_parameter(
            'n_objects', int(dcfg.get('n_objects', 4))
        ).value)
        self.min_area_px = float(self.declare_parameter(
            'min_area_px', float(dcfg.get('min_area_px', 25.0))
        ).value)
        self.jpeg_quality = int(self.declare_parameter(
            'jpeg_quality', int(dcfg.get('jpeg_quality', 95))
        ).value)

        self.num_images = num_images
        self.val_ratio = val_ratio
        self.out_dir = os.path.abspath(out_dir)
        self.img_train_dir = os.path.join(self.out_dir, 'images', 'train')
        self.img_val_dir = os.path.join(self.out_dir, 'images', 'val')
        self.lbl_train_dir = os.path.join(self.out_dir, 'labels', 'train')
        self.lbl_val_dir = os.path.join(self.out_dir, 'labels', 'val')
        for path in (self.img_train_dir, self.img_val_dir,
                     self.lbl_train_dir, self.lbl_val_dir):
            os.makedirs(path, exist_ok=True)

        # ---- 订阅：RGB / 内参 / 真值 ----
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._latest_image = None          # (np.ndarray(BGR), stamp_ns)
        self._latest_k = None              # 3x3 内参
        self._latest_gt = None             # ObjectStateArray
        self._frame_ready = threading.Event()

        self.create_subscription(Image, topics['image_raw'], self._on_image, qos,
                                 callback_group=group)
        self.create_subscription(CameraInfo, topics['camera_info'], self._on_info, qos,
                                 callback_group=group)
        self.create_subscription(ObjectStateArray, topics['ground_truth'], self._on_gt,
                                 qos, callback_group=group)

        # ---- TF：world -> camera_optical_frame（含域随机化后的真实相机位姿）----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_frame = cfg['frames']['world']
        self.camera_frame = cfg['frames']['camera_optical']

        # ---- 重置场景服务（每次采集前随机化）----
        self._reset_client = self.create_client(
            ResetScene, '/sorting/reset_scene', callback_group=group)

        self._running = True
        self._done = threading.Event()
        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'数据集采集启动：目标 {num_images} 张 -> {self.out_dir} '
            f'(val_ratio={val_ratio}, settle={self.settle_time}s)')

    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            if msg.encoding == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)[:, :, ::-1]
            elif msg.encoding == 'bgr8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            else:
                self.get_logger().warn(f'未支持的图像编码: {msg.encoding}',
                                       throttle_duration_sec=5.0)
                return
        except Exception as exc:
            self.get_logger().warn(f'图像解析失败: {exc}', throttle_duration_sec=5.0)
            return
        stamp_ns = msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec
        with self._lock:
            self._latest_image = (np.ascontiguousarray(arr), stamp_ns)
        self._frame_ready.set()

    def _on_info(self, msg: CameraInfo):
        k = np.array(msg.k, dtype=float).reshape(3, 3)
        with self._lock:
            self._latest_k = k

    def _on_gt(self, msg: ObjectStateArray):
        with self._lock:
            self._latest_gt = msg

    # ------------------------------------------------------------------
    def _call_reset(self, timeout: float = 90.0):
        """阻塞式调用 reset_scene（executor 在其它线程 spin，用 event 等结果）。"""
        if not self._reset_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/sorting/reset_scene 服务不可用')
            return False
        req = ResetScene.Request()
        req.n_objects = self.max_objects
        req.randomize_class = False     # "每种各一个"
        req.randomize_pose = True       # 类别->槽位随机排列
        req.randomize_scene = True      # 相机/光照域随机化
        done = threading.Event()
        future = self._reset_client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self.get_logger().error('reset_scene 调用超时')
            return False
        result = future.result()
        return bool(result and result.success)

    def _lookup_camera(self, stamp_ns: int):
        """取 world -> camera_optical_frame 变换，返回 (t_wc(3), R_wc(3x3))。"""
        stamp = rclpy.time.Time(nanoseconds=stamp_ns)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            # 时间戳精确匹配可能失败（TF 与图像不同源），退回"最新可用"
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, self.camera_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception as exc:
                self.get_logger().warn(f'相机 TF 不可用: {exc}', throttle_duration_sec=5.0)
                return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (np.array([t.x, t.y, t.z], dtype=float),
                _quat_to_matrix(q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------
    def _annotate(self, gt: ObjectStateArray, k: np.ndarray,
                  t_wc: np.ndarray, r_wc: np.ndarray,
                  width: int, height: int):
        """对一帧真值生成 YOLO-seg 标签行列表，返回 (lines, n_visible)。"""
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

        objects = []
        for state in gt.objects:
            size = np.array([state.size.x, state.size.y, state.size.z], dtype=float)
            pose: Pose = state.pose
            r_wo = _quat_to_matrix(pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w)
            t_wo = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            local = _surface_points(int(state.shape), size)
            world = local @ r_wo.T + t_wo
            # world -> optical:  p_o = R_wc^T (p_w - t_wc)
            optical = (world - t_wc) @ r_wc
            z = optical[:, 2]
            if np.all(z <= 1e-3):
                continue
            uv = np.column_stack([fx * optical[:, 0] / np.maximum(z, 1e-6) + cx,
                                  fy * optical[:, 1] / np.maximum(z, 1e-6) + cy])
            depth = float(np.mean(z[z > 1e-3]))
            objects.append({'class_id': int(state.class_id), 'uv': uv, 'depth': depth})

        if not objects:
            return [], 0

        # ---- 遮挡处理：近者优先占用像素 ----
        objects.sort(key=lambda o: o['depth'])
        buf = np.zeros((height, width), dtype=np.int32)
        for idx, obj in enumerate(objects, start=1):
            hull = cv2.convexHull(obj['uv'].astype(np.float32)).reshape(-1, 2)
            poly = np.round(hull).astype(np.int32)
            poly[:, 0] = np.clip(poly[:, 0], 0, width - 1)
            poly[:, 1] = np.clip(poly[:, 1], 0, height - 1)
            layer = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(layer, [poly], 255)
            free = (layer > 0) & (buf == 0)
            buf[free] = idx

        # ---- 提取每个物体可见区域的外轮廓 -> 多边形标签 ----
        lines = []
        n_visible = 0
        for idx, obj in enumerate(objects, start=1):
            mask = (buf == idx).astype(np.uint8) * 255
            if int(mask.sum()) // 255 < self.min_area_px:
                continue  # 几乎被完全遮挡，跳过该物体
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < self.min_area_px:
                continue
            approx = cv2.approxPolyDP(contour, 2.0, True).reshape(-1, 2)
            if len(approx) < 3:
                continue
            norm = approx.astype(float) / np.array([width, height], dtype=float)
            norm = np.clip(norm, 0.0, 1.0)
            coords = ' '.join(f'{v:.5f}' for v in norm.reshape(-1))
            lines.append(f'{obj["class_id"]} {coords}')
            n_visible += 1
        return lines, n_visible

    def _write_data_yaml(self):
        names = '\n'.join(f'  {c["id"]}: {c["name"]}' for c in self.class_table.classes)
        content = (
            f'# 由 dataset_capture 自动生成（YOLOv8-seg）\n'
            f'path: {self.out_dir}\n'
            f'train: images/train\n'
            f'val: images/val\n'
            f'names:\n{names}\n'
        )
        with open(os.path.join(self.out_dir, 'data.yaml'), 'w', encoding='utf-8') as fh:
            fh.write(content)

    # ------------------------------------------------------------------
    def _capture_loop(self):
        saved = 0
        attempts = 0
        max_attempts = self.num_images * 4
        try:
            while self._running and saved < self.num_images and attempts < max_attempts:
                attempts += 1
                if not self._call_reset():
                    time.sleep(2.0)
                    continue
                # 等待场景稳定 + 新的图像帧
                time.sleep(self.settle_time)
                self._frame_ready.clear()
                if not self._frame_ready.wait(timeout=10.0):
                    self.get_logger().warn('等待图像超时，重试')
                    continue

                with self._lock:
                    img = None if self._latest_image is None else self._latest_image[0]
                    stamp_ns = 0 if self._latest_image is None else self._latest_image[1]
                    k = None if self._latest_k is None else self._latest_k.copy()
                    gt = self._latest_gt
                if img is None or k is None or gt is None or not gt.objects:
                    time.sleep(1.0)
                    continue

                cam = self._lookup_camera(stamp_ns)
                if cam is None:
                    continue
                t_wc, r_wc = cam
                h, w = img.shape[:2]
                lines, n_visible = self._annotate(gt, k, t_wc, r_wc, w, h)
                if n_visible < len(gt.objects):
                    # 有物体没标注出来（严重遮挡/落在画外）——跳过该帧，避免噪声标签
                    self.get_logger().warn(
                        f'仅标注出 {n_visible}/{len(gt.objects)} 个物体，跳过该帧',
                        throttle_duration_sec=3.0)
                    continue

                name = f'frame_{saved:05d}'
                is_val = (saved % max(1, int(round(1.0 / max(self.val_ratio, 1e-6)))) == 0) \
                    if self.val_ratio > 0 else False
                img_dir = self.img_val_dir if is_val else self.img_train_dir
                lbl_dir = self.lbl_val_dir if is_val else self.lbl_train_dir
                cv2.imwrite(os.path.join(img_dir, name + '.jpg'), img,
                            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                with open(os.path.join(lbl_dir, name + '.txt'), 'w', encoding='utf-8') as fh:
                    fh.write('\n'.join(lines) + ('\n' if lines else ''))
                saved += 1
                if saved % 20 == 0 or saved == self.num_images:
                    self.get_logger().info(f'已采集 {saved}/{self.num_images} 张')
        except Exception as exc:
            self.get_logger().error(f'采集循环异常: {exc}')
        finally:
            self._write_data_yaml()
            self.get_logger().info(
                f'采集结束：共 {saved} 张，data.yaml 已写入 {self.out_dir}')
            self._done.set()

    def wait_done(self, timeout: float = None):
        return self._done.wait(timeout)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCaptureNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # 采集完成后自动退出（否则采集脚本会一直挂着等 Ctrl+C）
        while rclpy.ok() and not node._done.is_set():
            executor.spin_once(timeout_sec=0.2)
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
