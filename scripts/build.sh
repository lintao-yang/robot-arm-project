#!/bin/bash
# 构建 ROS 2 工作空间
#   用法: bash scripts/build.sh [包名 ...]
set -e

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash

cd "$WORKSPACE_DIR"

if [ "$#" -gt 0 ]; then
  PACKAGE_ARGS="--packages-select $*"
else
  PACKAGE_ARGS=""
fi

# DrvFs(/mnt/d) 不支持符号链接，因此不使用 --symlink-install
colcon build $PACKAGE_ARGS \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

echo
echo "构建完成。请执行: source install/setup.bash"