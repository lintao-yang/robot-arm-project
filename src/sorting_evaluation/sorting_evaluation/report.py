"""实验报告生成器。

读取 metrics_logger 输出的 statistics.csv，生成指标曲线与汇总报告：
    python3 -m sorting_evaluation.report --run-dir logs/<时间戳>
或  ros2 run sorting_evaluation make_report --run-dir logs/<时间戳>
"""

import argparse
import csv
import json
import os
import sys


def _load_rows(run_dir: str):
    csv_path = os.path.join(run_dir, 'statistics.csv')
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f'找不到 {csv_path}')
    with open(csv_path, 'r', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def build_report(run_dir: str, output_dir: str = None) -> dict:
    rows = _load_rows(run_dir)
    if not rows:
        return {'error': '统计文件为空'}

    output_dir = output_dir or run_dir
    os.makedirs(output_dir, exist_ok=True)

    elapsed = [_to_float(row['elapsed']) for row in rows]
    success_rate = [_to_float(row['grasp_success_rate']) for row in rows]
    accuracy = [_to_float(row['sorting_accuracy']) for row in rows]
    error_mm = [_to_float(row['mean_position_error_mm']) for row in rows]
    cycle_time = [_to_float(row['avg_cycle_time']) for row in rows]

    last = rows[-1]
    summary = {
        'run_dir': run_dir,
        'samples': len(rows),
        'duration_s': round(elapsed[-1], 2),
        'total_targets': int(_to_float(last['total_targets'])),
        'attempted': int(_to_float(last['attempted'])),
        'picked_success': int(_to_float(last['picked_success'])),
        'placed_correct': int(_to_float(last['placed_correct'])),
        'grasp_failures': int(_to_float(last['grasp_failures'])),
        'retries': int(_to_float(last['retries'])),
        'closed_loop_corrections': int(_to_float(last['closed_loop_corrections'])),
        'grasp_success_rate': round(_to_float(last['grasp_success_rate']), 4),
        'sorting_accuracy': round(_to_float(last['sorting_accuracy']), 4),
        'avg_cycle_time': round(_to_float(last['avg_cycle_time']), 4),
        'mean_position_error_mm': round(_to_float(last['mean_position_error_mm']), 3),
    }

    with open(os.path.join(output_dir, 'report.json'), 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    # ---- 绘图（matplotlib 缺失时降级为仅输出 JSON）----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(2, 2, figsize=(11, 7))

        axes[0][0].plot(elapsed, [v * 100 for v in success_rate], color='#1f6feb')
        axes[0][0].set_title('Grasp Success Rate (%)')
        axes[0][0].set_xlabel('time (s)')
        axes[0][0].grid(alpha=0.3)

        axes[0][1].plot(elapsed, [v * 100 for v in accuracy], color='#2da44e')
        axes[0][1].set_title('Sorting Accuracy (%)')
        axes[0][1].set_xlabel('time (s)')
        axes[0][1].grid(alpha=0.3)

        axes[1][0].plot(elapsed, error_mm, color='#bf3989')
        axes[1][0].set_title('Mean 3D Localization Error (mm)')
        axes[1][0].set_xlabel('time (s)')
        axes[1][0].grid(alpha=0.3)

        axes[1][1].plot(elapsed, cycle_time, color='#9a6700')
        axes[1][1].set_title('Average Cycle Time (s)')
        axes[1][1].set_xlabel('time (s)')
        axes[1][1].grid(alpha=0.3)

        figure.tight_layout()
        plot_path = os.path.join(output_dir, 'metrics.png')
        figure.savefig(plot_path, dpi=130)
        plt.close(figure)
        summary['plot'] = plot_path
    except Exception as exc:  # pragma: no cover
        summary['plot_error'] = str(exc)

    with open(os.path.join(output_dir, 'report.md'), 'w', encoding='utf-8') as handle:
        handle.write('# 分拣实验结果\n\n')
        handle.write(f'- 实验目录：`{run_dir}`\n')
        handle.write(f'- 任务耗时：{summary["duration_s"]} s\n')
        handle.write(f'- 待分拣目标：{summary["total_targets"]}\n')
        handle.write(f'- 尝试抓取：{summary["attempted"]}，成功 {summary["picked_success"]}\n')
        handle.write(f'- 抓取成功率：{summary["grasp_success_rate"] * 100:.1f}%\n')
        handle.write(f'- 分拣准确率：{summary["sorting_accuracy"] * 100:.1f}%\n')
        handle.write(f'- 平均单次周期：{summary["avg_cycle_time"]:.2f} s\n')
        handle.write(f'- 平均三维定位误差：{summary["mean_position_error_mm"]:.1f} mm\n')
        handle.write(f'- 二次视觉校正次数：{summary["closed_loop_corrections"]}\n')
        handle.write(f'- 重试次数：{summary["retries"]}\n')

    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description='生成分拣实验报告')
    parser.add_argument('--run-dir', required=True, help='metrics_logger 输出目录')
    parser.add_argument('--output-dir', default=None, help='报告输出目录（默认同 run-dir）')
    arguments = parser.parse_args(argv)

    summary = build_report(arguments.run_dir, arguments.output_dir)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())