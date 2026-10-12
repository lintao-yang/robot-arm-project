# WSL2 → 裸机 Ubuntu 双系统 迁移指南

> 背景：WSLg 无法稳定转发 Gazebo Sim 的 3D 视口（Ogre2 与 WSLg 的 D3D12 驱动
> 不兼容，软件渲染又过慢），项目改用裸机 Ubuntu 双系统，以获得硬件渲染与实时 GUI。

## 1. 迁移决策与影响

| 项 | 结论 |
|---|---|
| 源码 / 脚本 / 配置 / 文档 | 全部保留，直接拷贝 |
| `build/ install/ log/` | **平台相关，必须删除后重建** |
| ROS 2 / Gazebo / MoveIt 环境 | 裸机为空系统，**必须整套重装** |
| 源码存放位置 | **必须放 ext4 原生分区**（如 `~/robot_sorting_project`） |
| AI 上下文记忆 | 存于 Windows 侧，需随项目带走（见 `AI_MEMORY_BACKUP.md`） |

## 2. 迁移步骤

### 2.1 拷贝项目到原生分区

NTFS 挂载盘（`/mnt/...`、`/media/...`）无 Linux 权限位，会导致 colcon 符号链接
安装失效、Gazebo 网格解析不稳，因此**务必拷到 ext4**：

```bash
cp -r /media/<你的盘>/llm_training/robot_sorting_project ~/robot_sorting_project
cd ~/robot_sorting_project
```

### 2.2 一键重建环境

```bash
bash scripts/setup_ubuntu_native.sh
```

脚本内容：配置 ROS 2 apt 源 → 安装 ROS 2 Humble 与全部依赖 → 清理旧构建产物
→ `colcon build`。功能包依赖清单与各功能包 `package.xml` 一一对应。

> 补充：点云预处理 / GraspNet 节点依赖 `<workspace>/.venv` 中的 torch 与 scikit-learn
> （由 launch 按节点注入，见 `sorting_bringup/venv_env.py`）。该 venv 需自行创建并安装依赖；
> **不要**把 venv 的 site-packages 全局加到 `PYTHONPATH`（venv 的 numpy 1.26 会让 MoveIt 启动失败）。

### 2.3 GPU 驱动（本项目改用裸机的核心动机）

```bash
sudo ubuntu-drivers autoinstall     # 或 sudo apt install nvidia-driver-550
sudo reboot
nvidia-smi                          # 应能列出 RTX 3060
```

### 2.4 渲染方式切换（重要）

`scripts/env.sh` 现已默认 `LIBGL_ALWAYS_SOFTWARE=0`（裸机硬件渲染）。
若遇 GPU 异常可临时回退软件渲染：`export LIBGL_ALWAYS_SOFTWARE=1`。
GPU 正常时，Gazebo Sim 的 3D 视口应能实时显示，且不会出现 CPU 过载。

## 3. 验证清单

```bash
source /opt/ros/humble/setup.bash
source ~/robot_sorting_project/install/setup.bash

# (1) 依赖包是否都装上
ros2 pkg list | grep -E "ros_gz|ign_ros2_control|moveit_resources_panda"

# (2) 完整系统启动（这次带 GUI）
bash ~/robot_sorting_project/scripts/run_sorting.sh gui:=true
# 预期: Gazebo Sim 窗口显示桌面/分拣箱/Panda 机械臂，无 OGRE 异常

# (3) 触发抓取
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}"

# (4) 一键验收（headless 启动 + 触发抓取 + 收集指标）
bash ~/robot_sorting_project/scripts/acceptance_run.sh 420
```

## 4. 已知注意事项

- **必须 Ubuntu 22.04**：对应 ROS 2 Humble。若装成 24.04 需换 Jazzy，代码需适配。
- **Gazebo 用 Fortress**：项目已从 Classic 迁移到 Gazebo Sim (Fortress)，勿再装 Classic。
- **环境变量**：`env.sh` 会设置 `GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share`，
  否则机器人模型缺失网格与碰撞几何（必设项，与 WSL 无关）。
- **仿真时钟**：需把 `/world/sorting_world/clock` 桥接并重映射到 `/clock`，
  否则 `use_sim_time` 节点的定时器会冻结。
- **DDS 残留**：`scripts/cleanup_sim.sh` 用于清理 FastDDS 共享内存段，双系统下同样适用。