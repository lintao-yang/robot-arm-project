"""任务调度与闭环状态机节点。

状态机（对应 teach.txt 第十八节"难点4"的建议）：
    IDLE -> DETECT -> SELECT -> PLAN_PICK -> EXECUTE -> VERIFY -> CHECK -> DONE / RETRY

调度策略：
    * 目标筛选：优先选择置信度高、距基座近（可达性好）、深度噪声小的目标
    * 抓取顺序：先抓孤立、易抓的目标，降低碰撞与遮挡风险
    * 失败处理：单个目标最多重试 max_retries_per_target 次，之后跳过

真值使用边界（重要）：
    /sorting/ground_truth 只用于**离线评估指标**（定位误差、分拣准确率、放置落点），
    绝不参与目标选择或运动生成。在线抓取链路完全由视觉结果驱动。
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import (
    DetectionArray,
    ObjectStateArray,
    SortingStatistics,
)
from sorting_interfaces.srv import (
    ExecutePickPlace,
    PlanGrasp,
    ResetScene,
    StartSorting,
)

STATE_IDLE = 'IDLE'
STATE_DETECT = 'DETECT'
STATE_SELECT = 'SELECT'
STATE_PLAN_PICK = 'PLAN_PICK'
STATE_EXECUTE = 'EXECUTE'
STATE_VERIFY = 'VERIFY'
STATE_CHECK = 'CHECK'
STATE_RETRY = 'RETRY'
STATE_DONE = 'DONE'


class TaskPlannerNode(Node):
    """分拣任务状态机。"""

    def __init__(self):
        super().__init__('task_planner_node')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        topics = self.system_config['topics']
        task_cfg = self.system_config['task']

        self.use_closed_loop = bool(task_cfg.get('use_closed_loop', True))
        self.max_retries = int(task_cfg.get('max_retries_per_target', 2))
        self.refine_tolerance = float(task_cfg.get('refine_tolerance', 0.015))
        self.settle_time = float(task_cfg.get('settle_time', 1.5))

        roi = self.system_config['scene'].get('workspace_roi', {})
        self.roi = (
            float(roi.get('x_min', 0.0)), float(roi.get('x_max', 1.0)),
            float(roi.get('y_min', -1.0)), float(roi.get('y_max', 1.0)),
        )

        self.group = ReentrantCallbackGroup()

        # ---- 状态 ----
        self._state = STATE_IDLE
        self._running = False
        self._worker = None
        self._lock = threading.Lock()

        self._latest_targets = None
        self._targets_seq = 0
        self._ground_truth = None

        self._attempts = {}
        self._localization_errors = []

        self.stats = {
            'total_targets': 0,
            'attempted': 0,
            'picked_success': 0,
            'placed_correct': 0,
            'grasp_failures': 0,
            'retries': 0,
            'closed_loop_corrections': 0,
        }
        self._cycle_times = []
        self._task_start_time = None

        # ---- 订阅 ----
        self.create_subscription(
            DetectionArray, topics['targets'], self._on_targets, 5,
            callback_group=self.group,
        )
        self.create_subscription(
            ObjectStateArray, topics['ground_truth'], self._on_ground_truth, 10,
            callback_group=self.group,
        )

        # ---- 发布 ----
        self.pub_statistics = self.create_publisher(
            SortingStatistics, topics['statistics'], 10
        )
        self.pub_status = self.create_publisher(String, topics['task_status'], 10)

        # ---- 服务 ----
        self.create_service(
            StartSorting, self.system_config['services']['start_sorting'],
            self.on_start_sorting, callback_group=self.group,
        )
        self.plan_client = self.create_client(
            PlanGrasp, self.system_config['services']['plan_grasp'],
            callback_group=self.group,
        )
        self.execute_client = self.create_client(
            ExecutePickPlace, self.system_config['services']['execute_pick_place'],
            callback_group=self.group,
        )
        self.reset_client = self.create_client(
            ResetScene, self.system_config['services']['reset_scene'],
            callback_group=self.group,
        )

        self.create_timer(0.5, self.publish_statistics, callback_group=self.group)

        self.get_logger().info('任务调度节点启动')

    # ==================================================================
    def _set_state(self, state: str):
        self._state = state
        message = String()
        message.data = state
        self.pub_status.publish(message)
        self.get_logger().info(f'[状态] {state}')

    def _on_targets(self, msg: DetectionArray):
        self._latest_targets = msg
        self._targets_seq += 1

    def _on_ground_truth(self, msg: ObjectStateArray):
        self._ground_truth = msg

    def _wait_fresh_targets(self, timeout: float = 5.0):
        seq0 = self._targets_seq
        deadline = time.time() + timeout
        while time.time() < deadline and self._targets_seq <= seq0 and rclpy.ok():
            time.sleep(0.05)
        return self._latest_targets

    def _call(self, client, request, timeout: float = 60.0):
        if not client.wait_for_service(timeout_sec=15.0):
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    # ==================================================================
    # 指标
    # ==================================================================
    def _update_localization_error(self, detection):
        """用真值评估视觉三维定位误差（仅用于评估，不参与控制）。"""
        if self._ground_truth is None or not detection.has_3d:
            return
        best = None
        best_distance = 1e9
        for obj in self._ground_truth.objects:
            if int(obj.class_id) != int(detection.class_id):
                continue
            dx = obj.pose.position.x - detection.position_base.x
            dy = obj.pose.position.y - detection.position_base.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < best_distance:
                best_distance = distance
                best = obj
        if best is None or best_distance > 0.12:
            return
        # 真值物体为几何中心，检测为顶面点，故只比较平面距离与高度差
        dz = (best.pose.position.z + best.size.z / 2.0) - detection.position_base.z
        self._localization_errors.append(math.sqrt(best_distance ** 2 + dz ** 2) * 1000.0)

    def _object_in_zone(self, obj) -> bool:
        """真值判据：物体是否落在其对应分拣区内。"""
        zone = self.class_table.placement_zone_of(obj.class_id)
        zone_pose = self.class_table.zone_pose(zone)
        dx = obj.pose.position.x - zone_pose[0]
        dy = obj.pose.position.y - zone_pose[1]
        half = 0.16 / 2.0
        return abs(dx) < half and abs(dy) < half and obj.pose.position.z < 0.12

    def _count_correct_placements(self) -> int:
        if self._ground_truth is None:
            return 0
        return sum(1 for obj in self._ground_truth.objects if self._object_in_zone(obj))

    def publish_statistics(self):
        message = SortingStatistics()
        message.header.stamp = self.get_clock().now().to_msg()

        with self._lock:
            message.total_targets = int(self.stats['total_targets'])
            message.attempted = int(self.stats['attempted'])
            message.picked_success = int(self.stats['picked_success'])
            message.grasp_failures = int(self.stats['grasp_failures'])
            message.retries = int(self.stats['retries'])
            message.closed_loop_corrections = int(self.stats['closed_loop_corrections'])
            cycle_times = list(self._cycle_times)
            start_time = self._task_start_time
            errors = list(self._localization_errors)

        message.placed_correct = int(self._count_correct_placements())
        message.grasp_success_rate = (
            float(message.picked_success) / message.attempted if message.attempted else 0.0
        )
        message.sorting_accuracy = (
            float(message.placed_correct) / message.total_targets
            if message.total_targets else 0.0
        )
        message.avg_cycle_time = float(np.mean(cycle_times)) if cycle_times else 0.0
        message.total_time = float(time.time() - start_time) if start_time else 0.0
        message.mean_position_error_mm = float(np.mean(errors)) if errors else 0.0
        message.current_target_index = int(message.attempted)
        message.current_class = ''
        message.state = self._state

        self.pub_statistics.publish(message)

    # ==================================================================
    # 服务
    # ==================================================================
    def on_start_sorting(self, request, response):
        if self._running:
            response.success = False
            response.message = '分拣任务已在运行中'
            return response

        self.use_closed_loop = bool(request.use_closed_loop)
        continuous = bool(request.continuous)

        with self._lock:
            self.stats = {key: 0 for key in self.stats}
            self._cycle_times.clear()
            self._localization_errors.clear()
            self._attempts.clear()
            self._task_start_time = time.time()
            if self._ground_truth is not None:
                self.stats['total_targets'] = len(self._ground_truth.objects)

        self._running = True
        self._worker = threading.Thread(
            target=self._run_loop, args=(continuous,), daemon=True
        )
        self._worker.start()

        response.success = True
        response.message = (
            f'分拣任务已启动（闭环校正={"开启" if self.use_closed_loop else "关闭"}, '
            f'{"连续" if continuous else "单次"}模式）'
        )
        return response

    # ==================================================================
    # 主循环
    # ==================================================================
    def _run_loop(self, continuous: bool):
        try:
            if not self.plan_client.wait_for_service(timeout_sec=30.0):
                self.get_logger().error('/sorting/plan_grasp 不可用，任务中止')
                self._set_state(STATE_IDLE)
                return
            if not self.execute_client.wait_for_service(timeout_sec=30.0):
                self.get_logger().error('/sorting/execute_pick_place 不可用，任务中止')
                self._set_state(STATE_IDLE)
                return

            cycles = 0
            while self._running and rclpy.ok():
                self._set_state(STATE_DETECT)
                targets = self._wait_fresh_targets(timeout=8.0)
                if targets is None or not targets.detections:
                    self.get_logger().info('未检测到目标，任务结束')
                    break

                for detection in targets.detections:
                    if detection.has_3d:
                        self._update_localization_error(detection)

                self._set_state(STATE_SELECT)
                selected = self._select_target(targets)
                if selected is None:
                    self.get_logger().info('没有可抓取目标（已全部尝试或不可达）')
                    break

                success = self._execute_target(selected)
                cycles += 1
                if not continuous:
                    break
                if not success:
                    continue

            self._set_state(STATE_DONE if not self._running else STATE_CHECK)
            self._running = False
        except Exception as exc:  # 保证异常不吞掉状态
            self.get_logger().error(f'任务循环异常: {exc}')
            self._running = False
            self._set_state(STATE_IDLE)

    def _target_key(self, detection) -> str:
        return f'{int(detection.class_id)}_{detection.position_base.x:.2f}_{detection.position_base.y:.2f}'

    def _in_workspace(self, detection) -> bool:
        """目标是否落在机械臂工作区 ROI 内（分拣箱在 ROI 之外）。"""
        x_min, x_max, y_min, y_max = self.roi
        x = detection.position_base.x
        y = detection.position_base.y
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _select_target(self, targets: DetectionArray):
        """按"可达性 + 置信度 + 深度质量"综合评分选择目标。"""
        candidates = []
        for detection in targets.detections:
            if not detection.has_3d:
                continue
            if not self._in_workspace(detection):
                continue
            key = self._target_key(detection)
            attempts = self._attempts.get(key, {}).get('count', 0)
            if attempts > self.max_retries:
                continue

            reach = math.sqrt(
                detection.position_base.x ** 2 + detection.position_base.y ** 2
            )
            if reach > 0.78:
                continue

            score = (
                0.45 * float(np.clip(detection.confidence, 0, 1))
                + 0.25 * float(np.clip(1.0 - detection.depth_stddev / 0.02, 0, 1))
                + 0.30 * float(np.clip(1.0 - reach / 0.78, 0, 1))
                - 0.15 * attempts
            )
            candidates.append((score, detection, key))

        if not candidates:
            return None
        candidates.sort(key=lambda item: -item[0])
        return candidates[0][1], candidates[0][2]

    def _execute_target(self, selected) -> bool:
        detection, key = selected
        entry = self._attempts.setdefault(key, {'count': 0, 'failed': False})

        self._set_state(STATE_PLAN_PICK)
        plan = self._call(
            self.plan_client, PlanGrasp.Request(target=detection), timeout=20.0
        )
        if plan is None or not plan.success:
            self.get_logger().warn(
                f'抓取规划失败: {plan.message if plan else "服务无响应"}'
            )
            entry['count'] += 1
            entry['failed'] = True
            with self._lock:
                self.stats['grasp_failures'] += 1
            self._set_state(STATE_RETRY)
            return False

        with self._lock:
            self.stats['attempted'] += 1
            entry['count'] += 1

        self._set_state(STATE_EXECUTE)
        request = ExecutePickPlace.Request()
        request.candidate = plan.candidate
        request.use_closed_loop = self.use_closed_loop
        request.pregrasp_height = 0.0
        result = self._call(self.execute_client, request, timeout=180.0)

        self._set_state(STATE_VERIFY)
        if result is None:
            self.get_logger().error('运动执行服务无响应')
            entry['failed'] = True
            with self._lock:
                self.stats['grasp_failures'] += 1
            return False

        with self._lock:
            self._cycle_times.append(float(result.duration))
            if result.grasped:
                self.stats['picked_success'] += 1
            else:
                self.stats['grasp_failures'] += 1
            if result.retries:
                self.stats['retries'] += int(result.retries)
            if '二次校正' in result.message:
                self.stats['closed_loop_corrections'] += 1

        self.get_logger().info(
            f'{detection.class_name} 抓取={result.grasped} '
            f'放置复核={result.place_verified} 耗时={result.duration:.2f}s | {result.message}'
        )

        if result.grasped:
            entry['failed'] = False
            time.sleep(self.settle_time)
            return True

        entry['failed'] = True
        self._set_state(STATE_RETRY)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
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