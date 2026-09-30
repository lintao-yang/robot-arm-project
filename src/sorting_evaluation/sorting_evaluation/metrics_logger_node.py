"""模块7 日志与评价节点。

订阅 /sorting/statistics，按固定间隔采样并落盘：
    logs/<时间戳>/statistics.csv   —— 指标时间序列（可画曲线）
    logs/<时间戳>/summary.json     —— 任务结束/最近一次快照的汇总
    logs/<时间戳>/task_status.log  —— 状态机状态迁移记录
    logs/latest.json               —— 指向最新一次实验目录，便于脚本读取
"""

import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sorting_bringup import load_system_config
from sorting_interfaces.msg import SortingStatistics

CSV_FIELDS = [
    'timestamp', 'elapsed', 'state', 'total_targets', 'attempted',
    'picked_success', 'placed_correct', 'grasp_failures', 'retries',
    'closed_loop_corrections', 'grasp_success_rate', 'sorting_accuracy',
    'avg_cycle_time', 'mean_position_error_mm',
]


class MetricsLoggerNode(Node):
    """分拣过程指标记录。"""

    def __init__(self):
        super().__init__('metrics_logger_node')

        self.system_config = load_system_config()
        eval_cfg = self.system_config.get('evaluation', {})
        self.declare_parameter('log_dir', eval_cfg.get('log_dir', 'logs'))
        self.declare_parameter('sample_period', 1.0)

        base_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        if not os.path.isabs(base_dir):
            base_dir = os.path.join(os.getcwd(), base_dir)
        self.run_dir = os.path.join(base_dir, time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(self.run_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, 'statistics.csv')
        self.summary_path = os.path.join(self.run_dir, 'summary.json')
        self.status_path = os.path.join(self.run_dir, 'task_status.log')

        self._csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._csv_file.flush()

        self._status_file = open(self.status_path, 'w', encoding='utf-8')
        self._start = time.time()
        self._latest = None

        topics = self.system_config['topics']
        self.create_subscription(
            SortingStatistics, topics['statistics'], self._on_statistics, 10
        )
        self.create_subscription(String, topics['task_status'], self._on_status, 10)

        sample_period = float(self.get_parameter('sample_period').value)
        self.create_timer(sample_period, self._sample)

        os.makedirs(base_dir, exist_ok=True)
        with open(os.path.join(base_dir, 'latest.json'), 'w', encoding='utf-8') as handle:
            json.dump(
                {
                    'run_dir': self.run_dir,
                    'csv': self.csv_path,
                    'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                },
                handle,
                ensure_ascii=False,
                indent=2,
            )

        self.get_logger().info(f'指标记录已启动，输出目录: {self.run_dir}')

    # ------------------------------------------------------------------
    def _on_statistics(self, msg: SortingStatistics):
        self._latest = msg

    def _on_status(self, msg: String):
        self._status_file.write(f'{time.time() - self._start:.3f}\t{msg.data}\n')
        self._status_file.flush()

    def _row(self) -> dict:
        msg = self._latest
        if msg is None:
            return None
        return {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'elapsed': round(time.time() - self._start, 3),
            'state': msg.state,
            'total_targets': msg.total_targets,
            'attempted': msg.attempted,
            'picked_success': msg.picked_success,
            'placed_correct': msg.placed_correct,
            'grasp_failures': msg.grasp_failures,
            'retries': msg.retries,
            'closed_loop_corrections': msg.closed_loop_corrections,
            'grasp_success_rate': round(float(msg.grasp_success_rate), 4),
            'sorting_accuracy': round(float(msg.sorting_accuracy), 4),
            'avg_cycle_time': round(float(msg.avg_cycle_time), 4),
            'mean_position_error_mm': round(float(msg.mean_position_error_mm), 4),
        }

    def _sample(self):
        row = self._row()
        if row is None:
            return
        self._writer.writerow(row)
        self._csv_file.flush()

        with open(self.summary_path, 'w', encoding='utf-8') as handle:
            json.dump(row, handle, ensure_ascii=False, indent=2)

    def destroy_node(self):
        try:
            self._csv_file.close()
            self._status_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
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