#!/bin/bash
# 启动 rosbridge WebSocket 服务，供 Windows 侧 Foxglove Studio 实时可视化
# （WSLg 无法稳定转发 Gazebo 3D 视口，这是项目的实时可视化主通道）
#
#   用法:
#     1. WSL 终端 A: bash scripts/run_sorting.sh gui:=false
#     2. WSL 终端 B: bash scripts/run_visualization.sh
#     3. Windows: 打开 Foxglove Studio -> "打开连接" -> Rosbridge -> ws://localhost:9090
#        推荐面板: 3D(机器人+TF) / Image(/camera/image_raw 与 /sorting/detection_image)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
