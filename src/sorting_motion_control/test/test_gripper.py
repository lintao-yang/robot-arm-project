"""夹爪控制器的纯逻辑单元测试（节点交互用 Mock 隔离）。"""

from unittest.mock import MagicMock

from sorting_motion_control.gripper import GripperController


def _make_gripper():
    node = MagicMock()
    config = {
        'command_topic': '/panda_hand_controller/commands',
        'joint_names': ['panda_finger_joint1', 'panda_finger_joint2'],
        'state_joint': 'panda_finger_joint1',
        'open_effort': 6.0,
        'close_effort': 16.0,
        'effort_sign': -1.0,
        'open_position': 0.038,
        'grasp_threshold': 0.006,
        'command_settle_time': 0.0,
        'max_close_time': 0.0,
    }
    return GripperController(node, config)


def test_is_grasped_typical_object():
    gripper = _make_gripper()
    assert gripper.is_grasped(0.025) is True   # 夹住约 5cm 宽物体


def test_is_grasped_rejects_empty_close():
    gripper = _make_gripper()
    assert gripper.is_grasped(0.0) is False    # 空夹闭合到底
    assert gripper.is_grasped(0.004) is False  # 低于阈值
    assert gripper.is_grasped(None) is False   # 读不到关节状态


def test_is_grasped_rejects_no_movement():
    gripper = _make_gripper()
    assert gripper.is_grasped(0.038) is False  # 停在全开位 = 压根没动
    assert gripper.is_grasped(0.04) is False   # 被撑到全开 = 滑落


def test_hold_closing_publishes_signed_effort():
    gripper = _make_gripper()
    gripper.hold_closing()
    publisher = gripper._publisher
    publisher.publish.assert_called_once()
    message = publisher.publish.call_args[0][0]
    assert list(message.data) == [-16.0, -16.0]
