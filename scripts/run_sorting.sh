#!/bin/bash
# 一键启动完整分拣系统
#   用法: bash scripts/run_sorting.sh [gui:=false] [rviz:=true] [vision_backend:=classical]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"

export DISPLAY="${DISPLAY:-:0}"

cd "$WORKSPACE_DIR"
exec ros2 launch sorting_bringup sorting_system.launch.py "$@"