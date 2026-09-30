"""模块3 目标定位节点。

链路（对应 teach.txt 的"坐标链"）：
    像素坐标 (u, v) + 深度 z
      -> 相机光学系:  X = (u-cx)z/fx,  Y = (v-cy)z/fy,  Z = z
      -> TF2 外参:    P_base = T(base<-camera_optical) * P_camera
      -> 发布 /sorting/targets（带三维定位结果的 DetectionArray）

深度采用窗口内中位数滤波，抑制仿真深度噪声与边缘飞点。
"""

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

from sorting_bringup import load_system_config
from sorting_interfaces.msg import DetectionArray


class PoseEstimatorNode(Node):
    """像素 + 深度 -> 机械臂基坐标系三维位置。"""

    def __init__(self):
        super().__init__('pose_estimator_node')

        self.system_config = load_system_config()
        topics = self.system_config['topics']
        self.frames = self.system_config['frames']
        vision_cfg = self.system_config['vision']

        self.declare_parameter('depth_window', int(vision_cfg.get('depth_window', 9)))
        self.declare_parameter('min_depth', float(vision_cfg.get('min_depth', 0.2)))
        self.declare_parameter('max_depth', float(vision_cfg.get('max_depth', 2.0)))
        self.declare_parameter('use_mask_centroid', True)
        self.declare_parameter(
            'min_object_height', float(vision_cfg.get('min_object_height', 0.012))
        )

        self.depth_window = int(self.get_parameter('depth_window').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.use_mask_centroid = bool(self.get_parameter('use_mask_centroid').value)
        # 桌面高度 + 最小目标高度：低于该高度的三维点视为桌面/地面噪声
        self.table_top_z = float(self.system_config['scene']['table']['top_z'])
        self.min_object_height = float(self.get_parameter('min_object_height').value)
        self.min_object_z = self.table_top_z + self.min_object_height

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._depth_image = None
        self._camera_info = None
        self._depth_to_meters = 1.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, topics['depth_image'], self.on_depth, sensor_qos
        )
        self.create_subscription(
            CameraInfo, topics['camera_info'], self.on_camera_info, sensor_qos
        )
        self.create_subscription(
            DetectionArray, topics['detections'], self.on_detections, 5
        )

        self.pub_targets = self.create_publisher(DetectionArray, topics['targets'], 5)

        self._logged_frames = False
        self._dropped = 0
        self.get_logger().info('目标定位节点启动')

    # ------------------------------------------------------------------
    def on_depth(self, msg: Image):
        try:
            encoding = msg.encoding
            if encoding in ('32FC1', '64FC1'):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                self._depth_to_meters = 1.0
            elif encoding in ('16UC1', 'mono16'):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self._depth_to_meters = 0.001  # 毫米 -> 米
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                self._depth_to_meters = 1.0
            self._depth_image = np.asarray(depth, dtype=np.float32) * self._depth_to_meters
        except Exception as exc:
            self.get_logger().warn(f'深度图转换失败({msg.encoding}): {exc}',
                                   throttle_duration_sec=5.0)

    def on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    # ------------------------------------------------------------------
    def _lookup_base_from_camera(self):
        """返回 T_base_camera（4x4），失败返回 None。"""
        target = self.frames['base']
        source = self.frames['camera_optical']
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), Duration(seconds=0.2)
            )
        except Exception as exc:
            self.get_logger().warn(
                f'TF 查询失败 ({target} <- {source}): {exc}', throttle_duration_sec=5.0
            )
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

    def _sample_depth(self, u: float, v: float):
        """窗口内有效深度中位数 + 标准差。"""
        depth = self._depth_image
        height, width = depth.shape[:2]
        half = self.depth_window // 2

        cx = int(round(u))
        cy = int(round(v))
        x0, x1 = max(0, cx - half), min(width, cx + half + 1)
        y0, y1 = max(0, cy - half), min(height, cy + half + 1)
        if x1 <= x0 or y1 <= y0:
            return None, 0.0

        window = depth[y0:y1, x0:x1].reshape(-1)
        valid = window[np.isfinite(window) & (window > self.min_depth) & (window < self.max_depth)]
        if valid.size < max(3, window.size // 10):
            return None, 0.0
        return float(np.median(valid)), float(np.std(valid))

    # ------------------------------------------------------------------
    def on_detections(self, msg: DetectionArray):
        depth = self._depth_image
        camera_info = self._camera_info
        if depth is None or camera_info is None:
            self.get_logger().warn('深度图或相机内参尚未就绪，跳过本帧',
                                   throttle_duration_sec=5.0)
            return

        base_from_camera = self._lookup_base_from_camera()
        if base_from_camera is None:
            return

        if not self._logged_frames:
            self.get_logger().info(
                f'相机内参 fx={camera_info.k[0]:.2f}, fy={camera_info.k[4]:.2f}, '
                f'cx={camera_info.k[2]:.2f}, cy={camera_info.k[5]:.2f}'
            )
            self._logged_frames = True

        fx = float(camera_info.k[0]) or 1.0
        fy = float(camera_info.k[4]) or 1.0
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        out = DetectionArray()
        out.header = msg.header

        for detection in msg.detections:
            if self.use_mask_centroid and detection.has_mask:
                u, v = float(detection.centroid_u), float(detection.centroid_v)
            else:
                u = 0.5 * (detection.x_min + detection.x_max)
                v = 0.5 * (detection.y_min + detection.y_max)

            z, stddev = self._sample_depth(u, v)
            detection.has_3d = False
            detection.depth_stddev = float(stddev)

            if z is not None:
                # 质心 -> 三维点
                point_camera = self._backproject(u, v, z, camera_info)
                point_base = base_from_camera @ np.array(
                    [point_camera[0], point_camera[1], point_camera[2], 1.0]
                )

                detection.position_camera.x = float(point_camera[0])
                detection.position_camera.y = float(point_camera[1])
                detection.position_camera.z = float(point_camera[2])
                detection.position_base.x = float(point_base[0])
                detection.position_base.y = float(point_base[1])
                detection.position_base.z = float(point_base[2])
                detection.has_3d = True

                # 深度过滤：三维点低于桌面则判为桌面/地面噪声，整条检测丢弃
                if float(point_base[2]) < self.min_object_z:
                    self._dropped += 1
                    if self._dropped % 50 == 1:
                        self.get_logger().debug(
                            f'丢弃低于桌面的检测 {detection.class_name} '
                            f'(z={float(point_base[2]):.3f} < {self.min_object_z:.3f})'
                        )
                    continue

                # 掩码主轴方向 -> 基坐标系 XY 平面内的朝向
                # 在图像上沿主轴取第二个像素点（同一深度），反投影后求方向
                offset = 25.0
                u2 = u + offset * np.cos(detection.yaw_image)
                v2 = v + offset * np.sin(detection.yaw_image)
                point2_camera = self._backproject(u2, v2, z, camera_info)
                point2_base = base_from_camera @ np.array(
                    [point2_camera[0], point2_camera[1], point2_camera[2], 1.0]
                )
                dx = float(point2_base[0]) - float(point_base[0])
                dy = float(point2_base[1]) - float(point_base[1])
                detection.yaw_base = float(np.arctan2(dy, dx)) if (abs(dx) + abs(dy)) > 1e-6 else 0.0
            else:
                detection.yaw_base = 0.0

            out.detections.append(detection)

        self.pub_targets.publish(out)

    @staticmethod
    def _backproject(u: float, v: float, z: float, camera_info):
        """像素 + 深度 -> 相机光学坐标系三维点（针孔模型）。"""
        fx = float(camera_info.k[0]) or 1.0
        fy = float(camera_info.k[4]) or 1.0
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
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