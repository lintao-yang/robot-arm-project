#!/bin/bash
# 诊断探针：等待系统就绪后采集关键证据
#   1. 仿真时钟/关节状态频率（RTF 证据）
#   2. gz 统计话题（真实 RTF）
#   3. 服务/动作可见性
#   4. 触发一次连续分拣，观察运动执行链路是否响应
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

echo '[probe] 等待系统启动 (90s)...'
sleep 90

echo '=== [1] /clock 频率 (采样 8s) ==='
timeout 8 ros2 topic hz /clock 2>&1 | head -4

echo '=== [2] /joint_states 频率 (采样 8s) ==='
timeout 8 ros2 topic hz /joint_states 2>&1 | head -4

echo '=== [3] gz RTF ==='
timeout 10 ign topic -e -t /world/sorting_world/stats -n 1 2>&1 | grep -E 'real_time_factor|sim_time' | head -4

echo '=== [4] 服务与动作可见性 ==='
ros2 service list 2>/dev/null | grep -E 'sorting|apply_planning' | head -8
ros2 action list 2>/dev/null | head -5

echo '=== [5] 触发连续分拣 ==='
timeout 30 ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}" 2>&1 | tail -3

echo '=== [6] 观察 150s 运动执行链路 ==='
sleep 150

echo '--- graspnet_pick_place 日志 ---'
grep -aE 'graspnet_pick_place_node' /tmp/diag_system.log | tail -20
echo '--- graspnet_planner 日志 ---'
grep -aE 'graspnet_planner_node' /tmp/diag_system.log | tail -12
echo '--- move_group 末 6 行 ---'
grep -aE 'move_group-5' /tmp/diag_system.log | tail -6
echo '--- 统计快照 ---'
timeout 6 ros2 topic echo /sorting/statistics --once 2>&1 | head -20
