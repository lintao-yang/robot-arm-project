#!/bin/bash
# 演示模式：启动完整分拣系统（headless）+ Web 可视化操作台
# 用法: bash scripts/demo_start.sh
#   然后浏览器打开 http://localhost:8777/web_ui/index.html
#   Ctrl+C 结束演示（自动清理所有仿真/ROS 进程与 DDS 共享内存残留）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

# 启动前清场（CLEANUP_EXCLUDE 防止误匹配本脚本自身）
CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" || true
sleep 2

cleanup() {
  nohup bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 &
  true
}
trap cleanup EXIT INT TERM

echo '============================================================='
echo ' 演示系统启动中（约 80s 就绪）'
echo ' 就绪后浏览器打开: http://localhost:8777/web_ui/index.html'
echo ' Ctrl+C 结束演示'
echo '============================================================='

cd "$WORKSPACE_DIR"
ros2 launch sorting_bringup demo_viz.launch.py
