"""实测机械臂"可抓包络"：对工作区做网格化竖直抓取位姿的无碰撞逆解探测。

用途：
    为 system.yaml 的 scene.grasp_envelope 提供**实测依据**——它必须严格小于机械臂
    真实可达范围。先启动 move_group（跑起完整系统，或单独 move_group.launch.py），
    再执行本脚本；把输出的"建议包络"写进 grasp_envelope，并保证
    scene.layout.scatter_region ⊂ grasp_envelope。

用法：
    source scripts/env.sh
    python3 scripts/probe_grasp_envelope.py [--step 0.02] [--z 0.12] [--margin 0.03]

输出：
    * ASCII 地图：'.' 可抓(含无碰撞逆解) / 'x' 不可达或有碰撞
    * 可抓区域 bbox 与"建议包络"（bbox 内缩 margin，比实测更严格）
"""

import argparse
import sys
import threading
from collections import Counter

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from sorting_motion_control.moveit_client import MoveGroupClient
from sorting_motion_control.graspnet_pick_place_node import _vertical_pose

JOINTS = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
          'panda_joint5', 'panda_joint6', 'panda_joint7']


def frange(start, stop, step):
    values = []
    v = start
    if step > 0:
        while v <= stop + 1e-9:
            values.append(round(v, 4))
            v += step
    else:
        while v >= stop - 1e-9:
            values.append(round(v, 4))
            v += step
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--step', type=float, default=0.04)
    parser.add_argument('--z', type=float, default=0.12,
                        help='探测的 link8 高度（world 系，米）')
    parser.add_argument('--ik-timeout', type=float, default=2.0,
                        help='单次 IK 求解时间预算（秒）。注意 sec 需 >=1，'
                             '否则会被 MoveIt 当成"无时限"导致不可行格点各耗数秒')
    parser.add_argument('--x-min', type=float, default=0.16)
    parser.add_argument('--x-max', type=float, default=0.60)
    parser.add_argument('--y-min', type=float, default=-0.30)
    parser.add_argument('--y-max', type=float, default=0.30)
    parser.add_argument('--margin', type=float, default=0.03,
                        help='建议包络相对可抓 bbox 的内缩安全余量（米）')
    args = parser.parse_args()

    rclpy.init()
    node = Node('probe_grasp_envelope')
    moveit = MoveGroupClient(node, group_name='panda_arm', ee_link='panda_link8',
                             base_frame='panda_link0', joint_names=JOINTS)
    # 必须后台自旋：MoveGroupClient 是同步阻塞封装（call_async + 轮询 future），
    # 若不 spin，服务响应无人处理 -> future 永不完成 -> 所有 IK 都返回 None（会误判全不可抓）。
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    if not moveit.wait_for_server(timeout=30.0):
        print('错误：/move_action 不可用，请先启动 move_group')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    xs = frange(args.x_min, args.x_max, args.step)
    ys = frange(args.y_max, args.y_min, -args.step)     # 从上到下打印
    print(f'可抓包络探测  z(link8)={args.z:.3f}m  step={args.step:.3f}m', flush=True)
    print(f'  列(x) 从左到右: {xs[0]:.3f} -> {xs[-1]:.3f}', flush=True)
    print(f'  行(y) 从上到下: {ys[0]:.3f} -> {ys[-1]:.3f}', flush=True)

    ok_cells = []
    errs = Counter()
    for y in ys:
        row = ''
        for x in xs:
            pose = _vertical_pose(float(x), float(y), float(args.z))
            # 单次调用（含无碰撞逆解）即可判定可抓性；两倍调用会让探测耗时翻倍
            safe = moveit.compute_ik(pose, avoid_collisions=True, timeout=args.ik_timeout)
            if safe is not None and safe['error'] == 1:
                row += '.'
                ok_cells.append((x, y))
            else:
                row += 'x'
                errs['服务不可用/超时' if safe is None else safe['error_name']] += 1
        print(f'{y:+.3f} {row}', flush=True)
    print('图例: . 可抓(含无碰撞逆解)   x 不可达/有碰撞', flush=True)
    print(f'不可行原因统计: {dict(errs)}', flush=True)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    if not ok_cells:
        print('未探测到任何可抓点：请确认 move_group 已启动、机械臂处于 home 位。')
        return 1

    xs_ok = [cell[0] for cell in ok_cells]
    ys_ok = [cell[1] for cell in ok_cells]
    bbox = (min(xs_ok), max(xs_ok), min(ys_ok), max(ys_ok))
    print('\n可抓区域 bbox (x_min,x_max,y_min,y_max) = '
          f'({bbox[0]:.3f}, {bbox[1]:.3f}, {bbox[2]:.3f}, {bbox[3]:.3f})')
    margin = args.margin
    suggested = (bbox[0] + margin, bbox[1] - margin, bbox[2] + margin, bbox[3] - margin)
    print(f'建议 grasp_envelope（bbox 内缩 {margin * 1000:.0f}mm，比实测更严格）:')
    print(f'  x_min: {suggested[0]:.3f}\n  x_max: {suggested[1]:.3f}\n'
          f'  y_min: {suggested[2]:.3f}\n  y_max: {suggested[3]:.3f}')
    print('注意：请保证 scene.layout.scatter_region ⊂ 上述包络。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
