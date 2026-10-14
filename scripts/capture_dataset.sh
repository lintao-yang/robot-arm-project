#!/bin/bash
# 采集 YOLOv8-seg 合成数据集（自动标注）
#   用法: bash scripts/capture_dataset.sh [--ros-args -p num_images:=200 ...]
# 依赖 .venv 中的 ultralytics/opencv（见 scripts/train_yolo_seg.sh 说明）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"

# 注入项目 venv 依赖（cv2 / numpy 等），与 sorting_system.launch.py 的 VENV_ENV 一致
export PYTHONPATH="$WORKSPACE_DIR/.venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"

cd "$WORKSPACE_DIR"
exec ros2 run sorting_perception dataset_capture \
  --ros-args -p use_sim_time:=true "$@"
