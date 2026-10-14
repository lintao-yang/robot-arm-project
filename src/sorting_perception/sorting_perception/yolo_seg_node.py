"""YOLOv8-seg 在线推理：对相机 RGB 出实例掩码，合成"逐像素类别标签图"。

为什么做成"标签图"而不是直接接管点云：
    本项目约束"四种物体每种各一个"，因此 **类别即实例**。把分割结果压成一张
    单通道标签图（0=背景，class_id+1=类别），下游点云预处理节点只需把 3D 点投影到
    像素、读出该点的类别，就能把 DBSCAN 因 1.5mm 紧邻而合并成团的簇按类别拆开。
    这比传自定义实例掩码消息更简单，且可用 rqt_image_view 直接肉眼检查。

坐标系一致性说明：
    标签图与点云同源自 Gazebo 同一台相机。预处理节点用**同一套**相机外参做
    world->optical->像素投影（正向与反向用同一外参，误差相互抵消），因此只要
    在线跑仿真时相机未被域随机化抖动（demo 默认不抖动），投影与图像严格对齐。

运行（需 venv 依赖，见 scripts/ 里的启动脚本）：
    ros2 run sorting_perception yolo_seg_node --ros-args -p use_sim_time:=true
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from sorting_bringup import load_system_config

# 各类别在叠加图上的调试配色（BGR）
_CLASS_COLORS = [
    (40, 40, 220),    # 0 red_cube    -> 红
    (220, 120, 30),   # 1 blue_cyl    -> 蓝
    (60, 200, 60),    # 2 green_bar   -> 绿
    (30, 210, 230),   # 3 yellow_box  -> 黄
]


class YoloSegNode(Node):
    """YOLOv8-seg 推理 -> 逐像素类别标签图。"""

    def __init__(self):
        super().__init__('yolo_seg_node')

        cfg = load_system_config()
        ycfg = cfg.get('yolo_seg', {})
        self.infer_period = float(ycfg.get('infer_period', 0.2))
        self.conf = float(ycfg.get('conf', 0.25))
        self.imgsz = int(ycfg.get('imgsz', 640))
        self.device = ycfg.get('device', 0)
        self.input_topic = ycfg.get('input_image', '/camera/image_raw')
        self.label_topic = ycfg.get('output_label', '/sorting/class_label_image')
        self.overlay_topic = ycfg.get('output_overlay', '/sorting/seg_overlay')

        weights = ycfg.get('weights', '')
        if weights and not os.path.isabs(weights):
            root = os.environ.get('WORKSPACE_DIR') or os.getcwd()
            weights = os.path.join(root, weights)
        self.model = None
        if weights and os.path.isfile(weights):
            try:
                from ultralytics import YOLO  # 延迟导入：无权重时不必加载 torch
                self.model = YOLO(weights)
                self.get_logger().info(f'YOLOv8-seg 模型已加载: {weights}')
            except Exception as exc:
                self.get_logger().error(f'加载 YOLO 模型失败: {exc}')
        else:
            self.get_logger().warn(
                f'未找到分割权重（{weights}），yolo_seg 节点空转；'
                '请先运行 scripts/capture_dataset.sh 与 scripts/train_yolo_seg.sh')

        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(Image, self.input_topic, self._on_image, qos)
        self.pub_label = self.create_publisher(Image, self.label_topic, 3)
        self.pub_overlay = self.create_publisher(Image, self.overlay_topic, 3)
        self._last_infer = 0.0
        self.get_logger().info(
            f'YOLO 推理节点启动: {self.input_topic} -> {self.label_topic}')

    # ------------------------------------------------------------------
    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding == 'rgb8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)[:, :, ::-1].copy()
        if msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        if msg.encoding in ('mono8', '8UC1'):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise ValueError(f'未支持的图像编码: {msg.encoding}')

    def _on_image(self, msg: Image):
        now = time.time()
        if self.model is None or (now - self._last_infer) < self.infer_period:
            return
        self._last_infer = now
        try:
            bgr = self._to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        device = 'cpu' if int(self.device) < 0 else int(self.device)
        results = self.model.predict(source=bgr, conf=self.conf, imgsz=self.imgsz,
                                     device=device, verbose=False)
        h, w = bgr.shape[:2]
        label = np.zeros((h, w), dtype=np.uint8)
        overlay = bgr.copy()
        n_inst = 0
        if results:
            res = results[0]
            if res.masks is not None and res.boxes is not None:
                cls = res.boxes.cls.detach().cpu().numpy().astype(int)
                conf = res.boxes.conf.detach().cpu().numpy()
                masks = res.masks.data.detach().cpu().numpy()  # (n, mh, mw)
                # 高置信度优先占用像素，避免重叠处类别抖动
                for i in np.argsort(-conf):
                    m = masks[i]
                    if m.shape != (h, w):
                        m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                    m = m > 0.5
                    cid = int(cls[i])
                    label[m] = cid + 1
                    color = _CLASS_COLORS[cid % len(_CLASS_COLORS)]
                    overlay[m] = (0.5 * overlay[m] + 0.5 * np.array(color)).astype(np.uint8)
                    n_inst += 1

        header = msg.header
        self.pub_label.publish(self._make_image(label, header, 'mono8'))
        self.pub_overlay.publish(self._make_image(overlay, header, 'bgr8'))
        self.get_logger().debug(f'分割出 {n_inst} 个实例')

    @staticmethod
    def _make_image(arr: np.ndarray, header, encoding: str) -> Image:
        out = Image()
        out.header = header
        out.height, out.width = arr.shape[:2]
        out.encoding = encoding
        out.is_bigendian = 0
        out.step = int(arr.strides[0])
        out.data = arr.tobytes()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
