"""视觉检测后端。

统一接口：输入 BGR 图像 -> 输出 RawDetection 列表。

包含两个后端：
  * YoloSegDetector      : YOLO11-seg 实例分割（ultralytics），训练完成后使用
  * ClassicalColorDetector: HSV 颜色 + 连通域分割，用于冷启动/对照
"""

from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np

from sorting_bringup import ClassTable


@dataclass
class RawDetection:
    """后端输出的原始检测结果（掩码为可选）。"""

    class_id: int
    class_name: str
    confidence: float
    bbox: tuple = (0.0, 0.0, 0.0, 0.0)      # x_min, y_min, x_max, y_max
    mask: Optional[np.ndarray] = None        # uint8 0/1，与原图同尺寸
    centroid: tuple = (0.0, 0.0)             # (u, v) 像素
    area: float = 0.0
    yaw_image: float = 0.0
    elongation: float = 1.0                  # 主轴 / 次轴 特征值比
    extra: dict = field(default_factory=dict)


def mask_features(mask: np.ndarray) -> dict:
    """从二值掩码提取质心、面积、主轴方向与伸长度。"""
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return {'centroid': (0.0, 0.0), 'area': 0.0, 'yaw': 0.0, 'elongation': 1.0}

    pts = np.stack([xs, ys], axis=1).astype(np.float32)
    centroid = pts.mean(axis=0)

    area = float(xs.size)
    yaw = 0.0
    elongation = 1.0
    if pts.shape[0] >= 5:
        centered = pts - centroid
        # 2x2 协方差矩阵的特征分解 = PCA 主成分
        cov = np.cov(centered, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        major = eigvecs[:, order[0]]
        yaw = float(np.arctan2(major[1], major[0]))
        minor_var = max(float(eigvals[order[1]]), 1e-6)
        elongation = float(eigvals[order[0]] / minor_var)

    return {
        'centroid': (float(centroid[0]), float(centroid[1])),
        'area': area,
        'yaw': yaw,
        'elongation': elongation,
    }


class BaseDetector:
    """检测后端基类。"""

    name = 'base'

    def __init__(self, class_table: ClassTable, params: dict):
        self.class_table = class_table
        self.params = params
        self.min_mask_area = int(params.get('min_mask_area', 120))
        # 面积上限：用于剔除分拣箱 / 桌面色块等"非目标"大区域
        self.max_mask_area = int(params.get('max_mask_area', 0))

    def _area_ok(self, area: float) -> bool:
        if area < self.min_mask_area:
            return False
        if self.max_mask_area > 0 and area > self.max_mask_area:
            return False
        return True

    def detect(self, bgr: np.ndarray) -> List[RawDetection]:
        raise NotImplementedError

    def close(self):
        pass


class YoloSegDetector(BaseDetector):
    """YOLO11 实例分割后端（支持 .pt 分割/检测模型）。"""

    name = 'yolo'

    def __init__(self, class_table: ClassTable, params: dict):
        super().__init__(class_table, params)
        from ultralytics import YOLO  # 延迟导入，避免非 yolo 后端付出加载开销

        model_path = params.get('model_path') or ''
        if not model_path:
            raise ValueError('vision.model_path 为空，无法加载 YOLO 模型')
        self.model = YOLO(model_path)
        self.conf = float(params.get('conf_threshold', 0.45))
        self.iou = float(params.get('iou_threshold', 0.5))
        self.device = str(params.get('device', '')) or None
        self.max_det = int(params.get('max_detections', 20))
        self.is_seg = 'seg' in str(getattr(self.model, 'task', ''))
        # 使用模型自带的类别名，回落到 config 中的类别表
        names = getattr(self.model, 'names', {}) or {}
        self.model_names = {int(k): str(v) for k, v in names.items()}

    def detect(self, bgr: np.ndarray) -> List[RawDetection]:
        results = self.model.predict(
            source=bgr,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            max_det=self.max_det,
            verbose=False,
        )
        detections: List[RawDetection] = []
        if not results:
            return detections

        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return detections

        masks = result.masks.data.cpu().numpy() if getattr(result, 'masks', None) is not None else None
        h, w = bgr.shape[:2]

        for index in range(len(boxes)):
            cls_id = int(boxes.cls[index].item())
            conf = float(boxes.conf[index].item())
            xyxy = boxes.xyxy[index].cpu().numpy().astype(float)
            class_name = self.model_names.get(cls_id, self.class_table.name_of(cls_id))

            detection = RawDetection(
                class_id=cls_id,
                class_name=class_name,
                confidence=conf,
                bbox=(float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])),
            )

            if masks is not None and index < len(masks):
                mask = masks[index]
                # 模型输出掩码可能与原图尺寸不同，需还原
                if mask.shape[0] != h or mask.shape[1] != w:
                    mask = cv2.resize(mask.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)
                mask = (mask > 0.5).astype(np.uint8)
                if int(mask.sum()) < self.min_mask_area:
                    continue
                feat = mask_features(mask)
                if not self._area_ok(feat['area']):
                    continue
                detection.mask = mask
                detection.centroid = feat['centroid']
                detection.area = feat['area']
                detection.yaw_image = feat['yaw']
                detection.elongation = feat['elongation']
            else:
                # 纯检测模型：用框中心作为抓取参考点
                detection.centroid = ((xyxy[0] + xyxy[2]) / 2.0, (xyxy[1] + xyxy[3]) / 2.0)
                detection.area = float((xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1]))

            detections.append(detection)

        return detections

    def close(self):
        self.model = None


class ClassicalColorDetector(BaseDetector):
    """HSV 颜色分割后端（冷启动 / 对照组）。

    根据 classes.yaml 中每个类别的 rgba 推算色相范围，做连通域分割。
    """

    name = 'classical'

    def __init__(self, class_table: ClassTable, params: dict):
        super().__init__(class_table, params)
        self.hue_tolerance = int(params.get('hue_tolerance', 12))
        self.min_saturation = int(params.get('min_saturation', 70))
        self.min_value = int(params.get('min_value', 50))
        self._ranges = []
        for cls in class_table.classes:
            rgba = cls.get('rgba', [0.5, 0.5, 0.5, 1.0])
            # rgba 为 [r, g, b, a]，OpenCV 的 HSV 转换要求 BGR 顺序
            bgr_pixel = np.uint8([[[
                int(rgba[2] * 255), int(rgba[1] * 255), int(rgba[0] * 255)
            ]]])
            hsv = cv2.cvtColor(bgr_pixel, cv2.COLOR_BGR2HSV)[0][0]
            hue = int(hsv[0])
            self._ranges.append((int(cls['id']), str(cls['name']), hue))

    def _hue_distance(self, hue_a: int, hue_b: int) -> int:
        diff = abs(hue_a - hue_b)
        return min(diff, 180 - diff)

    def detect(self, bgr: np.ndarray) -> List[RawDetection]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h_channel, s_channel, v_channel = cv2.split(hsv)

        valid = (s_channel >= self.min_saturation) & (v_channel >= self.min_value)
        candidates = np.where(valid)
        if candidates[0].size == 0:
            return []

        # 每个有效像素归属到色相最近的类别
        hues = h_channel[candidates].astype(np.int32)
        best_class = np.full(hues.shape, -1, dtype=np.int32)
        best_dist = np.full(hues.shape, 10_000, dtype=np.int32)
        for class_id, _name, hue in self._ranges:
            dist = np.abs(hues - hue)
            dist = np.minimum(dist, 180 - dist)
            better = dist < best_dist
            best_dist = np.where(better, dist, best_dist)
            best_class = np.where(better, class_id, best_class)

        keep = best_dist <= self.hue_tolerance
        detections: List[RawDetection] = []

        for class_id, class_name, _hue in self._ranges:
            class_pixels = keep & (best_class == class_id)
            if not np.any(class_pixels):
                continue

            class_mask = np.zeros(h_channel.shape, dtype=np.uint8)
            class_mask[candidates[0][class_pixels], candidates[1][class_pixels]] = 1

            class_mask = cv2.morphologyEx(
                class_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
            )
            class_mask = cv2.morphologyEx(
                class_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
            )

            num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(
                class_mask, connectivity=8
            )
            for label in range(1, num_labels):
                area = int(stats[label, cv2.CC_STAT_AREA])
                if not self._area_ok(area):
                    continue
                x = int(stats[label, cv2.CC_STAT_LEFT])
                y = int(stats[label, cv2.CC_STAT_TOP])
                w = int(stats[label, cv2.CC_STAT_WIDTH])
                h = int(stats[label, cv2.CC_STAT_HEIGHT])
                mask = (labels == label).astype(np.uint8)
                feat = mask_features(mask)

                # 置信度用掩码填充度近似（规则物体填充度高）
                fill_ratio = area / float(max(w * h, 1))
                detections.append(
                    RawDetection(
                        class_id=class_id,
                        class_name=class_name,
                        confidence=float(np.clip(0.5 + 0.5 * fill_ratio, 0.0, 1.0)),
                        bbox=(float(x), float(y), float(x + w), float(y + h)),
                        mask=mask,
                        centroid=feat['centroid'],
                        area=feat['area'],
                        yaw_image=feat['yaw'],
                        elongation=feat['elongation'],
                    )
                )

        return detections


def build_detector(class_table: ClassTable, params: dict) -> BaseDetector:
    """按配置构造检测后端。"""
    backend = str(params.get('backend', 'yolo')).lower()
    if backend == 'classical':
        return ClassicalColorDetector(class_table, params)
    if backend == 'yolo':
        try:
            return YoloSegDetector(class_table, params)
        except Exception as exc:  # 模型缺失时退回颜色分割，保证链路可运行
            import rclpy.logging as _log

            _log.get_logger('detector_backend').warning(
                f'加载 YOLO 模型失败({exc})，回退到 classical 颜色分割后端'
            )
            return ClassicalColorDetector(class_table, params)
    raise ValueError(f'未知的视觉后端: {backend}')