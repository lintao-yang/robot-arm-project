"""system.yaml 配置完整性与重复键检测。

重复键教训：YAML 规范允许同名键（后者静默覆盖前者），曾导致 tcp_length
被定义两次而无人察觉。该测试用自定义 Loader 让重复键直接报错。
"""

from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'system.yaml'


class _UniqueKeyLoader(yaml.SafeLoader):
    """遇到重复键即抛错的 Loader。"""


def _construct_mapping_no_dupes(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if key in mapping:
            raise ValueError(f'system.yaml 存在重复键: {key!r}')
        mapping[key] = loader.construct_object(value_node, deep=True)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping_no_dupes
)


def _load():
    with open(CONFIG_PATH, 'r', encoding='utf-8') as handle:
        return yaml.load(handle, Loader=_UniqueKeyLoader)


def test_no_duplicate_keys():
    _load()  # 有重复键会直接抛错


def test_required_sections_present():
    config = _load()
    for section in ('frames', 'camera', 'topics', 'services', 'scene',
                    'grasp', 'motion', 'gripper', 'vision', 'task', 'evaluation'):
        assert section in config, f'缺少配置段: {section}'


def test_gripper_params_sane():
    gripper = _load()['gripper']
    assert gripper['close_effort'] > 0
    assert 0.0 <= gripper['grasp_threshold'] < gripper['open_position'] <= 0.04


def test_class_table_zone_poses():
    from sorting_bringup import ClassTable

    table = ClassTable()
    config = _load()
    for zone in config['scene']['bins']:
        pose = table.zone_pose(zone)
        assert len(pose) >= 2
