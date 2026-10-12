#!/bin/bash
# 一键启动完整分拣系统
#   用法: bash scripts/run_sorting.sh [gui:=false] [rviz:=true]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"

export DISPLAY="${DISPLAY:-:0}"

# 启动前清场（CLEANUP_EXCLUDE 防止误匹配本脚本自身）
CLEANUP_EXCLUDE=$ bash "$WORKSPACE_DIR/scripts/cleanup_sim.sh" || true
sleep 2

# 退出（Ctrl+C 或正常结束）时再清一次，避免残留累积成僵尸仿真
cleanup() {
  nohup bash "$WORKSPACE_DIR/scripts/cleanup_sim.sh" >/dev/null 2>&1 &
  true
}
trap cleanup EXIT INT TERM

cd "$WORKSPACE_DIR"
ros2 launch sorting_bringup sorting_system.launch.py "$@"