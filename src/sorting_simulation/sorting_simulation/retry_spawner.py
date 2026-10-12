"""带重试的 controller spawner。

解决 gz_ros2_control 异步加载 controller 导致的竞态：
spawner 调用 list_controllers 时 controller 已在列表中，
但 configure_controller 时 controller_manager 报 "no controller with this name exists"。

用法：
    ros2 run sorting_simulation retry_spawner --controller panda_arm_controller --max-retries 5
"""

import subprocess
import sys
import time


def main():
    import argparse

    parser = argparse.ArgumentParser(description='带重试的 controller spawner')
    parser.add_argument('--controller', required=True, help='控制器名称')
    parser.add_argument('--max-retries', type=int, default=5, help='最大重试次数')
    parser.add_argument('--retry-delay', type=float, default=5.0, help='重试间隔（秒）')
    # launch 的 Node 会自动附加 --ros-args 等 ROS 参数，需忽略未识别项
    args, _unknown = parser.parse_known_args()

    cmd = [
        'ros2', 'run', 'controller_manager', 'spawner',
        args.controller,
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '90',
    ]

    for attempt in range(1, args.max_retries + 1):
        print(f'[retry_spawner] 尝试 {attempt}/{args.max_retries}: {args.controller}')
        result = subprocess.run(cmd, capture_output=False)
        if result.returncode == 0:
            print(f'[retry_spawner] {args.controller} 配置成功')
            return 0
        if attempt < args.max_retries:
            print(f'[retry_spawner] 配置失败，{args.retry_delay}s 后重试...')
            time.sleep(args.retry_delay)

    print(f'[retry_spawner] {args.controller} 配置失败（已重试 {args.max_retries} 次）')
    return 1


if __name__ == '__main__':
    sys.exit(main())
