#!/bin/bash
# 诊断启动：后台拉起完整系统（headless），系统保持运行供交互式探针使用。
# 配套：diag_probe.sh（探针） -> cleanup_sim.sh（收尾）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

nohup ros2 launch sorting_bringup sorting_system.launch.py gui:=false rviz:=false \
  > /tmp/diag_system.log 2>&1 &
echo $! > /tmp/diag_launch.pid
echo "system launching (pid $(cat /tmp/diag_launch.pid)), log: /tmp/diag_system.log"
