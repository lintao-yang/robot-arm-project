"""sorting_vision：模块2 视觉感知（YOLO11-seg 检测 + 合成数据生成）。"""

from .detectors import (
    BaseDetector,
    ClassicalColorDetector,
    RawDetection,
    YoloSegDetector,
    build_detector,
    mask_features,
)

__all__ = [
    'BaseDetector',
    'ClassicalColorDetector',
    'RawDetection',
    'YoloSegDetector',
    'build_detector',
    'mask_features',
]