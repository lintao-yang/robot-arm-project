"""模块2 视觉感知节点。

订阅 RGB 图像，运行实例分割 / 检测后端，发布 sorting_interfaces/DetectionArray。
同时发布带可视化叠加的调试图（/sorting/detection_image）。
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import Detection, DetectionArray

from .detectors import build_detector

# 调色板：按 class_id 取色，便于可视化区分
_PALETTE = [
    (60, 60, 235),
    (235, 120, 60),
    (70, 200, 90),
    (40, 200, 230),
    (200, 80, 200),
    (200, 200, 60),
    (120, 120, 120),
]


class DetectorNode(Node):
    """视觉检测 ROS 2 节点。"""

    def __init__(self):
        super().__init__('detector_node')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        vision_cfg = dict(self.system_config.get('vision', {}))

        # 允许通过 ROS 参数覆盖关键项
        self.declare_parameter('backend', vision_cfg.get('backend', 'yolo'))
        self.declare_parameter('model_path', vision_cfg.get('model_path', ''))
        self.declare_parameter('device', vision_cfg.get('device', ''))
        self.declare_parameter('conf_threshold', float(vision_cfg.get('conf_threshold', 0.45)))
        self.declare_parameter('publish_debug_image', True)

        vision_cfg['backend'] = self.get_parameter('backend').value
        vision_cfg['model_path'] = self.get_parameter('model_path').value
        vision_cfg['device'] = self.get_parameter('device').value
        vision_cfg['conf_threshold'] = self.get_parameter('conf_threshold').value

        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.get_logger().info(f'构建视觉后端: {vision_cfg["backend"]}')
        self.detector = build_detector(self.class_table, vision_cfg)
        self.get_logger().info(f'视觉后端就绪: {self.detector.name}')

        topics = self.system_config['topics']
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_image = self.create_subscription(
            Image, topics['image_raw'], self.on_image, sensor_qos
        )
        self.pub_detections = self.create_publisher(
            DetectionArray, topics['detections'], 10
        )
        self.pub_debug_image = self.create_publisher(
            Image, topics['detection_image'], 2
        )

        self._frame_count = 0
        self.get_logger().info(
            f'检测节点启动: 订阅 {topics["image_raw"]} -> 发布 {topics["detections"]}'
        )

    # ------------------------------------------------------------------
    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败: {exc}', throttle_duration_sec=5.0)
            return

        raw_detections = self.detector.detect(bgr)
        self._frame_count += 1

        out = DetectionArray()
        out.header = msg.header
        for raw in raw_detections:
            out.detections.append(self._to_msg(raw, msg.header))

        self.pub_detections.publish(out)

        if self.publish_debug and self._frame_count % 3 == 0:
            self._publish_debug(bgr, out, msg.header)

    # ------------------------------------------------------------------
    def _to_msg(self, raw, header) -> Detection:
        msg = Detection()
        msg.header = header
        msg.class_id = int(raw.class_id)
        msg.class_name = raw.class_name
        msg.confidence = float(raw.confidence)
        msg.x_min, msg.y_min, msg.x_max, msg.y_max = [float(v) for v in raw.bbox]
        msg.has_mask = raw.mask is not None
        msg.centroid_u = float(raw.centroid[0])
        msg.centroid_v = float(raw.centroid[1])
        msg.mask_area = float(raw.area)
        msg.yaw_image = float(raw.yaw_image)
        msg.elongation = float(raw.elongation)
        msg.has_3d = False
        return msg

    def _publish_debug(self, bgr, detections: DetectionArray, header):
        canvas = bgr.copy()
        for det in detections.detections:
            color = _PALETTE[int(det.class_id) % len(_PALETTE)]
            cv2.rectangle(
                canvas,
                (int(det.x_min), int(det.y_min)),
                (int(det.x_max), int(det.y_max)),
                color,
                1,
            )
            label = f'{det.class_name} {det.confidence:.2f}'
            cv2.putText(
                canvas, label,
                (int(det.x_min), max(12, int(det.y_min) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
            )
            if det.has_mask:
                cu, cv_ = int(det.centroid_u), int(det.centroid_v)
                cv2.drawMarker(canvas, (cu, cv_), color, cv2.MARKER_CROSS, 10, 2)
                # 画出掩码主轴方向（用于长条物体的抓取 yaw 估计）
                length = 22
                dx = int(length * np.cos(det.yaw_image))
                dy = int(length * np.sin(det.yaw_image))
                cv2.line(canvas, (cu - dx, cv_ - dy), (cu + dx, cv_ + dy), color, 1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_image.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f'调试图发布失败: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detector.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()