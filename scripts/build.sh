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

# --symlink-install：install/ 中的 Python 代码软链接回 src/，
# 改完 .py 保存后重启节点即生效，无需重新 colcon build（"边改边仿真"）。
# 历史背景：WSL 时代源码位于 DrvFs(/mnt/d)，不支持符号链接故未启用；
# 迁移到 ext4 原生分区后该限制消失，恢复启用。
# 如需关闭（例如排查环境差异），可 export NO_SYMLINK_INSTALL=1。
if [ -n "${NO_SYMLINK_INSTALL:-}" ]; then
  SYMLINK_ARGS=""
else
  SYMLINK_ARGS="--symlink-install"
fi

colcon build $PACKAGE_ARGS $SYMLINK_ARGS \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

echo
echo "构建完成。请执行: source install/setup.bash"