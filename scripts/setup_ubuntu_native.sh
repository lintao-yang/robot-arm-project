#!/bin/bash
# 裸机 Ubuntu 22.04 环境重建脚本（WSL2 -> 双系统迁移后使用）
#   用法: bash scripts/setup_ubuntu_native.sh [--no-build]
#   前置: 已装 Ubuntu 22.04；且已把项目拷到 ext4 原生分区
#         （勿放在 /mnt 或 /media 的 NTFS 挂载盘上）
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> [0/5] 校验系统版本"
. /etc/os-release
if [ "${VERSION_ID}" != "22.04" ]; then
  echo "警告: 当前 Ubuntu ${VERSION_ID}，本项目要求 22.04（对应 ROS 2 Humble）"
  echo "      若为 24.04 需改用 ROS 2 Jazzy，代码需另行适配。"
fi

echo "==> [1/5] 配置 ROS 2 apt 源"
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> [2/5] 安装 ROS 2 Humble 桌面版与构建工具"
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools python3-colcon-common-extensions

echo "==> [3/5] 安装仿真/控制/规划/可视化依赖"
# ros-humble-ros-gz-sim 会拉入 Ignition/Gazebo Fortress；
# ign-ros2-control 即 Fortress 下的 gz_ros2_control（真实力矩驱动）
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
  ros-humble-ign-ros2-control \
  ros-humble-moveit ros-humble-moveit-resources-panda-description ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-rosbridge-suite ros-humble-web-video-server \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller ros-humble-effort-controllers \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-tf2-ros \
  ros-humble-tf-transformations ros-humble-cv-bridge ros-humble-image-transport \
  python3-numpy python3-opencv python3-yaml python3-matplotlib python3-pip

echo "==> [4/5] 清理历史构建产物（平台相关，必须删除重建）"
rm -rf "$PROJECT_DIR/build" "$PROJECT_DIR/install" "$PROJECT_DIR/log"

echo "==> [5/5] 构建工作空间"
if [ "${1:-}" != "--no-build" ]; then
  bash "$PROJECT_DIR/scripts/build.sh"
fi

echo
echo "==========================================================="
echo " 完成。接下来:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source $PROJECT_DIR/install/setup.bash"
echo "   bash $PROJECT_DIR/scripts/run_sorting.sh gui:=true"
echo " 注意: 点云预处理/GraspNet 节点依赖 <workspace>/.venv 中的"
echo "       torch 与 scikit-learn（由 launch 按节点注入，见 sorting_bringup/venv_env.py）"
echo "==========================================================="