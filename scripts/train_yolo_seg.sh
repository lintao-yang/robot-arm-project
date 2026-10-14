#!/bin/bash
# 训练 YOLOv8-seg 实例分割模型
#   用法: bash scripts/train_yolo_seg.sh [data.yaml] [epochs] [imgsz] [model]
#   默认: datasets/yolo_seg/data.yaml, 10 epochs, 640, yolov8n-seg.pt
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

DATA="${1:-$WORKSPACE_DIR/datasets/yolo_seg/data.yaml}"
EPOCHS="${2:-10}"
IMGSZ="${3:-640}"
MODEL="${4:-yolov8n-seg.pt}"

if [ ! -f "$DATA" ]; then
  echo "数据集配置不存在: $DATA（请先运行 scripts/capture_dataset.sh）" >&2
  exit 1
fi

cd "$WORKSPACE_DIR"
exec "$WORKSPACE_DIR/.venv/bin/yolo" segment train \
  model="$MODEL" data="$DATA" epochs="$EPOCHS" imgsz="$IMGSZ" \
  device=0 batch=8 patience=50 \
  project="$WORKSPACE_DIR/runs/segment" name=sort_objects exist_ok=True
