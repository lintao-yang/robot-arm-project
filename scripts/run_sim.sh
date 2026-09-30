#!/bin/bash
# 启动仿真环境（Gazebo + Panda + 控制器 + 场景管理）
#   用法: bash scripts/run_sim.sh [gui:=true|false] [...]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"

# WSLg 提供 X 显示，使 gzserver 可以创建离屏渲染上下文
export DISPLAY="${DISPLAY:-:0}"

cd "$WORKSPACE_DIR"
exec ros2 launch sorting_simulation sim_bringup_gz.launch.py "$@"