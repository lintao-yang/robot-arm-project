"""共享配置加载工具。

所有功能包统一通过本模块读取 ``sorting_bringup`` 中安装到 ``share/`` 的 YAML，
避免类别定义在多处重复维护。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sorting_bringup'


def load_yaml(relative_path: str) -> dict:
    """读取 sorting_bringup 包内的 YAML 文件。"""
    share_dir = get_package_share_directory(PACKAGE_NAME)
    path = os.path.join(share_dir, relative_path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'配置文件不存在: {path}')
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def load_classes() -> dict:
    """读取类别与分拣区定义（classes.yaml）。"""
    return load_yaml(os.path.join('config', 'classes.yaml'))


def load_system_config() -> dict:
    """读取全局配置（system.yaml）。"""
    return load_yaml(os.path.join('config', 'system.yaml'))


class ClassTable:
    """类别定义查询表，供视觉 / 抓取 / 任务节点复用。"""

    def __init__(self, classes_config: dict = None):
        data = classes_config if classes_config is not None else load_classes()
        self._classes = data.get('classes', [])
        self._zones = data.get('zones', {})
        self._dr = data.get('domain_randomization', {})
        self._by_id = {int(c['id']): c for c in self._classes}
        self._by_name = {c['name']: c for c in self._classes}

    # ---- 类别查询 ----
    @property
    def classes(self):
        return self._classes

    @property
    def class_ids(self):
        return sorted(self._by_id.keys())

    @property
    def class_names(self):
        return [c['name'] for c in self._classes]

    @property
    def zones(self):
        return self._zones

    @property
    def domain_randomization(self):
        return self._dr

    def get(self, class_id):
        return self._by_id.get(int(class_id))

    def get_by_name(self, name):
        return self._by_name.get(name)

    def name_of(self, class_id):
        item = self.get(class_id)
        return item['name'] if item else f'unknown_{class_id}'

    def size_of(self, class_id):
        item = self.get(class_id)
        return list(item['size']) if item else [0.04, 0.04, 0.04]

    def height_of(self, class_id):
        """物体沿世界 Z 轴的标称高度（米）。"""
        return float(self.size_of(class_id)[2])

    def grasp_width_of(self, class_id):
        item = self.get(class_id)
        return float(item['grasp_width']) if item else 0.04

    def shape_of(self, class_id):
        item = self.get(class_id)
        return item['shape'] if item else 'box'

    def use_orientation(self, class_id):
        item = self.get(class_id)
        return bool(item.get('use_orientation', False)) if item else False

    def rgba_of(self, class_id):
        item = self.get(class_id)
        return list(item.get('rgba', [0.7, 0.7, 0.7, 1.0])) if item else [0.7, 0.7, 0.7, 1.0]

    def placement_zone_of(self, class_id):
        item = self.get(class_id)
        return item['placement_zone'] if item else 'zone_a'

    def zone_pose(self, zone_name):
        zone = self._zones.get(zone_name, {})
        return list(zone.get('bin_pose', [0.66, 0.0, 0.0]))

    def zone_rgba(self, zone_name):
        zone = self._zones.get(zone_name, {})
        return list(zone.get('rgba', [0.5, 0.5, 0.5, 1.0]))