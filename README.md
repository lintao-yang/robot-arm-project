# 基于点云 6DoF 抓取检测与运动规划的机械臂无序抓取系统仿真

> ROS 2 Humble + MoveIt 2 + Gazebo Sim (Fortress) + RGB-D 点云 + GraspNet 6DoF + Franka Panda 两指夹爪

本项目是一个**感知—决策—规划—执行—反馈**完整闭环的机械臂无序抓取（bin picking）仿真系统：
顶置 RGB-D 相机采集桌面点云，点云预处理（ROI / 去桌面 / 聚类）后送入预训练 **GraspNet**
推理 6DoF 抓取位姿，经夹爪宽度与接近方向过滤后，由 MoveIt 2 规划并执行
"预抓取 → 下降 → 闭合 → 抬升 → 搬运 → 释放"的抓取闭环，
抓取确认基于**真实接触物理**（手指关节位置判据），最后输出量化评估指标。

> 说明：本项目曾有一条"2D 检测（YOLO/颜色分割）+ 垂直四段式抓取 + 按类别分拣"的旧链路，
> 现已**整体移除**。当前链路不含任何 2D 检测/YOLO 依赖，也不做按类别的分区放置，
> 所有物体抓取后投放至**单一投放点**（工业无序抓取惯例）。分类需求留待后续在点云层面迭代。

---

## 1. 快速开始

项目运行在 **Ubuntu 22.04 原生 ext4 分区**（勿放在 `/mnt`、`/media` 的 NTFS 挂载盘上，
否则 `colcon build` 的符号链接会失效、Gazebo 网格解析会异常）。

```bash
# 载入环境（ROS 2 Humble + 本工作空间 + Gazebo 资源路径）
source scripts/env.sh

# 构建（ext4 原生分区启用 --symlink-install：改完 .py 无需重新编译）
bash scripts/build.sh

# 一键启动完整系统（仿真 + MoveIt + 感知/抓取/执行/评估）
bash scripts/run_sorting.sh gui:=true

# 另开终端触发抓取（continuous: true 连续抓取，false 只抓一次）
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}"

# 查看实时指标
ros2 topic echo /sorting/statistics
```

重置场景（重新随机摆放物体）：

```bash
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 4, randomize_class: true, randomize_pose: true, randomize_scene: true}"
```

### 1.1 分层调试（推荐）

Gazebo 仿真与 MoveIt 常驻，改动算法代码只重启业务节点，无需重开仿真：

```bash
# 终端1（常驻）: 仿真世界
ros2 launch sorting_simulation sim_bringup_gz.launch.py gui:=true
# 终端2（常驻）: MoveIt 规划
ros2 launch sorting_simulation move_group.launch.py
# 终端3（改完就重启）: 业务节点（点云/抓取/执行/评估）
ros2 launch sorting_bringup sorting_nodes.launch.py
```

### 1.2 演示模式（Web 操作台）

headless 启动完整系统，并拉起 rosbridge（9090）、web_video_server（8080）与内置操作台（8777）：

```bash
bash scripts/demo_start.sh
# 就绪后浏览器打开 http://localhost:8777/web_ui/index.html
```

操作台包含：ros3djs 实时 3D 机械臂与抓取候选标记、相机画面、
抓取候选列表（评分 / 开口 / 位置）、任务状态与统计指标、事件日志，
以及"开始连续抓取 / 单次抓取 / 重置场景"按钮。

---

## 2. 系统架构

### 2.1 模块与功能包对应关系

| 模块 | 功能包 | 关键节点 / 文件 |
|---|---|---|
| 接口定义 | `sorting_interfaces` | `Grasp6D`、`Grasp6DArray`、`SortingStatistics`、`ObjectState(Array)`、`StartSorting`、`ResetScene` |
| 全局配置 | `sorting_bringup` | `config/classes.yaml`、`config/system.yaml`、顶层 launch、`venv_env` |
| 仿真环境 | `sorting_simulation` | `sorting_world_sim.sdf`、`panda_gz.urdf.xacro`、`scene_manager`、`move_group.launch.py` |
| 点云预处理 | `sorting_perception` | `pointcloud_preprocessor` |
| 6DoF 抓取检测 | `sorting_grasp_planning` | `graspnet_planner_node`（复用第三方 GraspNet） |
| 运动规划与控制 | `sorting_motion_control` | `moveit_client`、`gripper`、`graspnet_pick_place_node` |
| 日志与评价 | `sorting_evaluation` | `metrics_logger_node`、`report` |

第三方代码与权重位于 `third_party/graspnet-baseline/`（含 `checkpoint-rs.tar`），已加入 `.gitignore`。

### 2.2 数据流

```
Gazebo(RGB-D) ──/camera/points──▶ pointcloud_preprocessor（ROI + 去桌面 + DBSCAN 聚类）
                                        │
                        /sorting/workspace_cloud（相机光学系，仅物体簇点）
                                        ▼
                              graspnet_planner_node（GraspNet，GPU 推理）
                                        │  宽度过滤 + 接近方向过滤 + 时序累积 + 体素去重
                        /sorting/graspnet_grasps（world 系 6DoF 抓取数组）
                                        ▼
                       graspnet_pick_place_node ──▶ MoveIt /move_action + /compute_cartesian_path
                                        │              （夹爪 /panda_hand_controller/commands，effort 力控）
                                        ▼
                        /sorting/statistics + /sorting/task_status ──▶ metrics_logger_node
```

### 2.3 抓取执行流程

```
IDLE → DETECT（等待 GraspNet 抓取帧）→ EXECUTE（按评分逐个尝试）→ DONE
  每次尝试：预抓取点 → 沿 approach 直线下降 → 闭合夹爪并确认 → 沿 approach 直线抬升
            → 搬运到投放点 → 释放 → 退回
```

单轮按评分降序取前 `max_attempts` 个候选，逐个尝试；全部失败才判本轮抓取失败（连续模式下进入下一轮）。

---

## 3. 关键设计说明

### 3.1 坐标系约定

- **相机光学系**（z 前 / x 右 / y 下）：GraspNet 的训练约定，点云预处理输出到该系。
- **world 系**：抓取检测与运动规划的统一坐标系。仿真中机器人由 `ros_gz_sim` 以
  `(0,0,0)`、无旋转生成，因此 **`panda_link0` 与 `world` 完全重合**，world 系抓取可直接
  作为 MoveIt（base=`panda_link0`）的位姿目标。
- 相机安装位姿在 `system.yaml` 与 `sorting_world_sim.sdf` 中必须一致，否则三维定位会出现系统性偏差。

### 3.2 GraspNet 抓取姿态 → panda_link8 位姿映射

GraspNet 姿态约定（取自第三方源码，非推测）：
`utils/loss_utils.py::batch_viewpoint_params_to_matrix` 给出 R 的三列，其中 `col0 = approach`；
`graspnetAPI/utils/utils.py::plot_gripper_pro_max` 的指爪掩码表明两指沿 `col1` 分开，
即 `col1 = closing`、`col2 = height`。

映射到 Panda（`graspnet_pick_place_node.link8_orientation`）：
- `panda_link8` 的 +z 轴指向指尖，即抓取接近方向 → `R_link8[:,2] = approach`；
- 两指闭合方向在 link8 坐标系中恒为 `normalize(1,1,0)`（由 `panda_hand` 相对 `panda_link8`
  绕 z 的固定安装角决定）→ 约束 `R_link8 @ normalize(1,1,0) = closing 的垂直分量`。

该构造在 `approach=(0,0,-1)` 时**精确退化为**旧链路已验证的垂直下抓姿态
（`Rz(45°)·diag(1,-1,-1)`），可作单元回归。抓取中心沿 approach 后退 `tcp_offset` 即为
`panda_link8` 目标位置（GraspNet 的 translation 为两指之间的抓取中心）。

### 3.3 运动执行

- 大范围移动：MoveIt `/move_action` 位姿目标（OMPL + 碰撞检测 + KDL 逆解）
- 下降抓取 / 抬升退离 / 投放下降 / 返回：`/compute_cartesian_path` **直线插补**
  （优先），避免 OMPL 生成"先横移再下压"的绕行路径把物体顶飞；直线不可达时退化为位姿目标规划
- 规划场景中加入桌面与分拣箱碰撞体

### 3.4 夹爪力控（真实接触物理）

夹爪使用 `effort_controllers/JointGroupEffortController`（真实力控）：

| 指令 | 数值 | 说明 |
|---|---|---|
| 张开 | `+open_effort`（6 N） | 手指到 0.04（全开，总开口 0.08 m） |
| 夹持 | `effort_sign * close_effort`（-16 N） | 阶梯加载，轻触建立接触不撞偏物体 |
| 搬运保持 | `effort_sign * hold_effort`（-30 N） | 抓取确认后切换，抵抗搬运滑落 |
| 判定 | `grasp_threshold < pos < open_position - 0.006` | 排除"空夹到底"与"压根没动" |

Gazebo Sim 下机械臂由 `gz_ros2_control` 以**真实力矩驱动**，接触力反传到关节，
靠指尖摩擦即可夹持搬运，抓取确认与抬升后滑落检测均为真实物理量。

### 3.5 Python 依赖：按节点注入 .venv（重要）

项目 Python 依赖（torch / scikit-learn）装在 `<workspace>/.venv`，而 ROS 安装出的可执行脚本
shebang 指向 `/usr/bin/python3`（无这些依赖）。**不能**全局把 venv 的 site-packages 注入
`PYTHONPATH`：venv 的 numpy 1.26 已移除 `np.float` 等旧别名，会让 MoveIt 等仍依赖旧 numpy
的组件启动失败。因此由 `sorting_bringup/sorting_bringup/venv_env.py` 提供工具，
**只在需要 venv 的节点上**（点云预处理、GraspNet 推理）通过 `additional_env` 注入，其余节点保持不变。

---

## 4. 感知与抓取后端

### 4.1 点云预处理（`pointcloud_preprocessor`）

处理链（合成场景桌面占 ~99% 的点，且俯视相机存在机械臂自遮挡，必须先裁剪再去除）：

1. 坐标系统一：Gazebo body 系 → 相机光学系 + world 系
2. 工作区 ROI + 高度夹层：剔除桌面下方与高处机械臂自遮挡点
3. RANSAC 平面分割：剔除桌面平面
4. DBSCAN 欧式聚类：按点数与包围盒尺寸过滤，输出物体簇
5. 只发布**筛选后的物体簇点云**（而非去平面后的全部点，后者仍含料箱壁残片等杂点）

算法复用成熟开源库：scikit-learn（DBSCAN / RANSAC）、numpy。

### 4.2 GraspNet 6DoF 抓取检测（`graspnet_planner_node`）

复用第三方预训练 GraspNet（`third_party/graspnet-baseline`，权重 `checkpoint-rs.tar`），
GPU 推理。后处理：

- 评分阈值过滤 + **夹爪宽度过滤**（开口超过 78 mm 的抓取直接筛除）
- **接近方向过滤**：约束抓取接近方向与"竖直向下"的夹角不超过 `max_approach_deg`
  （GraspNet 训练于侧视堆叠场景，会提出大量侧向抓取，而本项目末端为下抓）
- **时序累积**：静态场景下单帧只对部分物体提出抓取，累积最近若干帧并按体素去重以补齐覆盖

---

## 5. 实验与指标

`metrics_logger_node` 输出到 `logs/<时间戳>/`：

- `statistics.csv` —— 指标时间序列（状态、尝试数、成功率、周期时间）
- `summary.json` —— 最近一次快照
- `task_status.log` —— 抓取状态迁移记录

生成报告与图表：

```bash
ros2 run sorting_evaluation make_report --run-dir logs/<时间戳>
# 输出 report.json / report.md / metrics.png
```

| 指标 | 计算方式 |
|---|---|
| 抓取成功率 | 抓取确认成功次数 / 尝试次数 |
| 平均周期时间 | 单次抓取-放置总耗时均值 |

> 注：旧链路的"三维定位误差 / 分拣准确率"依赖 2D 检测真值，已随旧链路移除。

---

## 6. 环境搭建要点（踩坑记录）

1. **官方 Panda 描述缺少 `<inertial>`** → sdformat 会丢弃所有 link，Gazebo 中生成空模型。
   解决：`urdf_utils.inject_inertials` 在 xacro 展开后注入 Franka 官方惯量参数。
2. **机械臂底座未固定会自由落体** → URDF 中加入名为 `world` 的 link 与固定关节，
   Gazebo 会把 `world` 识别为世界坐标系并锁死底座。
3. **`GZ_SIM_RESOURCE_PATH` 必须包含 ROS share 目录**：URDF 网格以 `model://<包名>/...`
   引用，缺失时机器人"没有可视与碰撞几何"。已在 `scripts/env.sh` 固化。
4. **Gazebo Sim 的时钟话题不在 `/clock`**：真正发布的是 `/world/<world>/clock`，
   `use_sim_time` 节点只认 `/clock`，需桥接 + 重映射，否则定时器冻结。
5. **URDF 与 SDF 的 `<box>` 语法不同**：URDF 用 `size` 属性，SDF 用 `<size>` 子元素。
   写错会让整个 collision 被静默丢弃（曾导致指尖"穿过"物体抓不住）。
6. **回调阻塞会耗尽 executor 线程池**：在定时器回调里阻塞等待服务会占满线程，
   响应回调用无线程可执行、请求永远超时。抓取循环因此放在独立线程中执行。
7. **ROS launch 的 `LaunchConfiguration` 对象恒为 truthy**：`if gui` 不会按实参分支，
   条件启动必须用 `IfCondition/UnlessCondition`。
8. **`kill -9` 会泄漏 FastDDS 共享内存段**（`/dev/shm/fastrtps_*`）：残留段会导致 DDS
   话题/服务投递不可靠。`scripts/cleanup_sim.sh` 统一清理进程与 SHM 段。
9. **Gazebo Sim (Fortress) 的进程名是 `ign gazebo` 而非 `gz sim`**：清理/trap 脚本
   用 `pkill -f 'gz sim'` 永远匹配不到，会泄漏僵尸仿真器。
10. **numpy 1.26 移除 `np.float` 等旧别名**：venv 依赖与 MoveIt 的系统 numpy 不可混用，
    故采用按节点注入（见 3.5）。

---

## 7. 目录结构

```
robot_sorting_project/
├── src/
│   ├── sorting_interfaces/          # 自定义消息与服务
│   ├── sorting_bringup/             # 全局配置 + 顶层 launch + venv 注入工具
│   ├── sorting_simulation/          # 世界 / URDF / 场景管理 / MoveIt 启动
│   ├── sorting_perception/          # 点云预处理
│   ├── sorting_grasp_planning/      # GraspNet 6DoF 抓取检测
│   ├── sorting_motion_control/      # MoveIt 接口 / 夹爪 / 抓取执行
│   └── sorting_evaluation/          # 指标记录与报告
├── third_party/graspnet-baseline/   # 第三方 GraspNet 代码与权重（git 忽略）
├── scripts/                         # env.sh / build.sh / run_sorting.sh / demo_start.sh ...
├── web_ui/                          # Web 操作台
├── docs/                            # 补充文档
└── logs/                            # 实验日志输出
```