#!/bin/bash
# 环境初始化（被其它脚本 source）
#   - 载入 ROS 2 Humble
#   - 载入本工作空间
#   - 设置 Gazebo Sim 渲染与资源路径变量（WSLg 下默认软件渲染）

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi

# 渲染方式：默认**强制软件渲染**（Mesa llvmpipe）。
# 原因（本机 WSLg + RTX 3060 混合显卡实测）：Gazebo Sim 的 Ogre2 走 D3D12 驱动时会在
#   OGRE EXCEPTION(9:UnimplementedException) in GL3PlusTextureGpu::copyTo
# 处抛异常并终止进程（GUI 与相机传感器都会触发）。改用软件渲染后 GUI 与 RGB-D
# 相机均稳定工作（相机 ~12 Hz）。
# 若你的机器 GPU 直通正常，可设为 0 以获得更好性能与画面质量。
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

# Gazebo Sim 资源路径：URDF 中的网格/模型以 model://<包名>/... 形式引用
# （sdformat 会把 package:// 转成 model://），必须把 ROS 的 share 目录加进来，
# 否则机器人会"没有可视与碰撞几何"（日志报 Unable to find file with URI
# [model://moveit_resources_panda_description/meshes/...]）。
export GZ_SIM_RESOURCE_PATH="/opt/ros/humble/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

export WORKSPACE_DIR