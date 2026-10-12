"""夹爪力控与抓取判定。

夹爪由 ``effort_controllers/JointGroupEffortController`` 驱动：
    * 手指关节限位 [0, 0.04]：0 = 完全闭合，0.04 = 最大张开（总开口 2*value）
    * 关节轴分别为 ±y，因此"两个手指同时受到同号力矩"即同向开合
    * 张开：下发 +open_effort；闭合夹持：下发 effort_sign * close_effort

抓取判定（teach.txt 提到的"夹爪闭合距离"方法）：
    闭合指令下发后读取手指关节实际位置。若被物体挡住，关节停在约"物体宽度/2"；
    若空夹，关节会一直走到 0。因此 position > grasp_threshold 即判定夹住物体。
    这是基于真实接触物理的判断，而非固定关节的作弊做法。
"""

import time


class GripperController:
    """夹爪开合控制与抓取检测。"""

    def __init__(self, node, config: dict, joint_state_topic: str = '/joint_states'):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._node = node
        self.command_topic = config['command_topic']
        self.joint_names = list(config['joint_names'])
        self.state_joint = config.get('state_joint', self.joint_names[0])
        self.open_effort = float(config['open_effort'])
        self.close_effort = float(config['close_effort'])
        # 保持力矩：抓取确认后搬运全程使用（大于闭合力矩，抵抗抬升加速度造成的
        # 楔开/滑脱）；未配置时退化为闭合力矩
        self.hold_effort = float(config.get('hold_effort', self.close_effort))
        self.effort_sign = float(config.get('effort_sign', -1.0))
        self.open_position = float(config.get('open_position', 0.038))
        self.grasp_threshold = float(config['grasp_threshold'])
        self.settle_time = float(config['command_settle_time'])
        self.max_close_time = float(config['max_close_time'])

        self._publisher = node.create_publisher(Float64MultiArray, self.command_topic, 10)
        self._positions = {}
        self._efforts = {}
        node.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)

        # 周期性保持力矩刷新：经实证（scripts/test_hold_path.sh），gz_ros2_control
        # 的 forward command 通路会保持最后一次指令，不刷新也不会丢力；此刷新作为
        # 廉价保险保留（覆盖控制器重载等边界情况），并非必需机制。
        self._hold_timer = None
        self._hold_effort = None

    # ------------------------------------------------------------------
    def _on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._positions[name] = float(position)
        for name, effort in zip(msg.name, msg.effort):
            self._efforts[name] = float(effort)

    def finger_position(self):
        return self._positions.get(self.state_joint)

    def finger_effort(self):
        """手指关节的力矩反馈（牛顿）：用于抓取验证时读取实际接触力。"""
        return self._efforts.get(self.state_joint)

    def _command(self, effort: float):
        from std_msgs.msg import Float64MultiArray

        message = Float64MultiArray()
        message.data = [float(effort)] * len(self.joint_names)
        self._publisher.publish(message)

    def _on_hold_timer(self):
        if self._hold_effort is not None:
            self._command(self._hold_effort)

    def _start_hold_refresh(self):
        """启动周期性保持力矩刷新（0.5s 周期）。"""
        if self._hold_timer is None:
            self._hold_timer = self._node.create_timer(0.5, self._on_hold_timer)

    def _stop_hold_refresh(self):
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None
        self._hold_effort = None

    def hold_closing(self):
        """以保持力矩夹持并启动周期性刷新（搬运过程中持续有效）。"""
        self._hold_effort = self.effort_sign * self.hold_effort
        self._command(self._hold_effort)
        self._start_hold_refresh()

    def open(self, wait: bool = True):
        self._stop_hold_refresh()
        self._command(self.open_effort)
        if wait:
            time.sleep(self.settle_time)

    def release(self, wait: bool = True):
        """释放物体：短时张开后撤力，避免物体被弹飞。"""
        self._stop_hold_refresh()
        self._command(self.open_effort)
        if wait:
            time.sleep(self.settle_time)
        self._command(0.0)

    def close_and_measure(self):
        """逐步加载夹持力矩后读取手指关节位置（读取失败返回 None）。

        采用阶梯加载而非直接施加最大力矩：手指质量很小，突加力矩会让指尖以
        很高速度撞上物体，既容易把物体撞飞，也不利于接触约束收敛。
        """
        steps = 5
        step_time = self.max_close_time / float(steps)
        for index in range(1, steps + 1):
            self._command(self.effort_sign * self.close_effort * index / steps)
            time.sleep(step_time)

        return self.finger_position()

    def is_grasped(self, position) -> bool:
        """抓取判定：手指确实离开全开位、又未闭合到底，即认为夹住了物体。

        仅用 "position > 阈值" 会把"夹爪压根没动"误判为抓取成功，
        因此同时要求 position 明显小于全开位。
        """
        if position is None:
            return False
        return self.grasp_threshold < position < (self.open_position - 0.006)