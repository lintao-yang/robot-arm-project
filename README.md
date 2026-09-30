# 基于视觉感知与运动规划的机械臂闭环分拣系统仿真

> ROS 2 Humble + MoveIt 2 + Gazebo Classic 11 + RGB-D 视觉 + Franka Panda 两指夹爪

本项目是一个**感知—决策—规划—执行—反馈**完整闭环的机械臂分拣仿真系统：相机采集桌面场景，
视觉模型识别目标类别与像素位置，结合深度图与 TF 外参恢复目标在机械臂基坐标系下的三维位置，
生成四段式抓取位姿，由 MoveIt 2 规划无碰撞轨迹并执行抓取，按类别放入对应分拣区，
并在**抓取前二次视觉校正、抓取确认、放置复核**三处实现闭环反馈，最后输出量化评估指标。

---

## 1. 快速开始

> 项目运行在 **WSL2 + Ubuntu 22.04** 中，虚拟磁盘位于 `D:\llm_training\wsl\`（不占用 C 盘）。
> 源码同时挂载在 Windows 侧 `D:\llm_training\robot_sorting_project`，可直接用编辑器修改。

```bash
# 进入 WSL
wsl -d Ubuntu-22.04

# 一次性依赖：抓取搬运用的模型吸附附件（Gazebo Classic 桥接，第三方现成库）
sudo apt install ros-humble-gazebo-model-attachment-plugin \
                 ros-humble-gazebo-model-attachment-plugin-msgs

# 载入环境（ROS 2 Humble + 本工作空间）
source /mnt/d/llm_training/robot_sorting_project/scripts/env.sh

# 构建
bash /mnt/d/llm_training/robot_sorting_project/scripts/build.sh

# 启动完整系统（gui:=true 可打开 Gazebo 界面）
bash /mnt/d/llm_training/robot_sorting_project/scripts/run_sorting.sh gui:=false

# 另开一个终端触发分拣（连续模式 + 开启闭环校正）
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: true, continuous: true}"

# 查看实时指标
ros2 topic echo /sorting/statistics
```

重置场景（重新随机摆放物体）：

```bash
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 4, randomize_class: true, randomize_pose: true, randomize_scene: true}"
```

---

## 2. 系统架构

### 2.1 模块与功能包对应关系

| teach.txt 模块 | 功能包 | 关键节点 / 文件 |
|---|---|---|
| — | `sorting_interfaces` | 自定义 msg/srv（Detection、GraspCandidate、SortingStatistics、ObjectState…） |
| — | `sorting_bringup` | 全局配置（classes.yaml / system.yaml）+ 顶层 launch |
| 模块1 仿真环境 | `sorting_simulation` | `sorting_world.world`、`panda_gazebo.urdf.xacro`、`scene_manager` |
| 模块2 视觉感知 | `sorting_vision` | `detector_node`、`detectors.py`（YOLO11-seg / 颜色分割）、`synthetic_dataset` |
| 模块3 目标定位 | `sorting_pose_estimation` | `pose_estimator_node`（像素+深度→相机系→基座系，TF2） |
| 模块4 抓取决策 | `sorting_grasp_planning` | `grasp_planner_node`（四段式抓取位姿） |
| 模块5 运动规划与控制 | `sorting_motion_control` | `moveit_client`、`gripper`、`motion_executor_node` |
| 模块6 闭环反馈 | 同上 + `sorting_task_planning` | 抓取前二次校正、抓取确认、放置复核 |
| 模块7 日志与评价 | `sorting_evaluation` | `metrics_logger_node`、`report` |

### 2.2 数据流

```
Gazebo(RGB-D) ──/camera/image_raw──▶ detector_node ──/sorting/detections──▶ pose_estimator_node
                                                                                    │
                                                        /sorting/targets (含 base 系三维位置)
                                                                                    │
                                                                                    ▼
 task_planner_node ──/sorting/plan_grasp──▶ grasp_planner_node ──GraspCandidate──▶ motion_executor_node
       │                                                                                    │
       │◀──────────────── /sorting/statistics ◀── metrics_logger_node                       │
       └──────────────── /sorting/execute_pick_place ────────────────────────────────────────┘
                                          （MoveIt /move_action + /compute_cartesian_path）
                                          （夹爪 /panda_hand_controller/commands，effort 力控）
```

### 2.3 闭环状态机

```
IDLE → DETECT → SELECT → PLAN_PICK → EXECUTE → VERIFY → CHECK → DONE / RETRY
```

闭环体现在三个层次：

1. **抓取前二次视觉校正**：机械臂到达预抓取点后重新检测目标，调用抓取规划服务重算抓取位姿
   （`motion_executor_node._refine_grasp`）。实测校正量可达毫米级。
2. **抓取确认**：夹爪闭合后读取手指关节实际位置，被物体挡住则判定夹住；抬升后再次判定以发现滑落
   （`gripper.is_grasped`）。这是基于接触物理的判断，不是"固定关节"作弊。
   > 已建立吸附后，抬升后的保持判据改为"物体是否仍随夹爪移动"（比较 `/gazebo/model_states`
   > 中物体与抓取点的水平距离）。原因见 3.5 节：吸附附件会改变手指与被吸附物体的接触关系，
   > 此时手指位置不再反映真实夹持状态。**闭合瞬间的接触判定仍为纯物理量。**
3. **放置后视觉复核**：释放并退回后重新检测，确认目标已进入对应分拣区
   （`motion_executor_node._verify_placement`）。

---

## 3. 关键设计说明

### 3.1 坐标系与标定链

```
像素 (u,v) + 深度 z
   → 相机光学系  X=(u-cx)z/fx, Y=(v-cy)z/fy, Z=z          （pose_estimator_node._backproject）
   → 机械臂基座系 P_base = T(base←camera_optical) · P_camera （TF2，由 scene_manager 广播）
```

`scene_manager` 依据相机安装位姿 + 标准光学旋转（body: x前/y左/z上 → optical: z前/x右/y下）
广播 `world → camera_optical_frame`；`world → panda_link0` 由 URDF 中的固定关节经
`robot_state_publisher` 发布。相机安装位姿在 `system.yaml` 与 `sorting_world.world` 中
必须保持一致，否则会产生系统性定位偏差。

### 3.2 抓取位姿生成

- 统一"末端垂直向下"两指夹取，姿态四元数 `q = q_z(yaw) · q_x(π)`
- 由 URDF 可知 `panda_hand` 相对 `panda_link8` 绕 z 旋转 -45°，手指沿 `panda_hand` 的 ±y 闭合，
  因此**夹爪闭合轴方位角 = yaw - 45°**，即 `yaw = 期望闭合方位角 + π/4`
- 长条物体（掩码主轴/次轴特征值比 > 2.5）使用掩码主方向，让夹爪垂直于长轴闭合
- 夹取高度由两个几何约束共同决定（`grasp_planner_node._grasp_height`）：
  - 手掌约束：`link8 - palm_offset ≥ 物体顶面 + 余量`（防止手掌碰撞体压到物体顶部）
  - 指尖约束：`link8 - tcp_length ≥ 物体底面 + 余量`（防止指尖扎进桌面）
  - 取两者较高者，在"手掌不碰物体"的前提下让手指尽量多地包住物体

### 3.3 运动执行

- 大范围移动：MoveIt `/move_action` 位姿目标（OMPL + 碰撞检测 + KDL 逆解）
- 下降抓取 / 垂直抬升 / 放置下降 / 抬升退回：`/compute_cartesian_path` **直线插补**，
  避免 OMPL 生成"先横移再抬升"的绕行路径把物体拖掉；时间轴按速度比例重定时
- 规划场景中加入桌面与 4 个分拣箱碰撞体

### 3.4 夹爪力控

夹爪使用 `effort_controllers/JointGroupEffortController`（真实力控）：

| 指令 | 数值 | 说明 |
|---|---|---|
| 张开 | `+open_effort` | 实测 +8 N 使手指到 0.04（全开，总开口 0.08 m） |
| 夹持 | `effort_sign * close_effort` | 实测 -8 N 使手指闭合到 0 |
| 判定 | `grasp_threshold < pos < open_position - 0.006` | 既排除"空夹到底"，也排除"压根没动" |

闭合采用阶梯加载力矩，避免指尖以高速冲击物体。

### 3.5 搬运保持（第三方吸附附件）

Gazebo Classic 下本机械臂走 `position` 接口，`gazebo_ros2_control` 对其调用
`Joint::SetPosition(...,true)` 做运动学跟随，机械臂没有真实动力学，**仅靠摩擦无法把物体带起来**
（实测：能夹住，抬升即脱出，见 `docs/KNOWN_ISSUES.md`）。

解决方案是引入社区通用做法——现成的第三方库
[`gazebo_model_attachment_plugin`](https://github.com/Boeing/gazebo_model_attachment_plugin)
（ROS 2 Humble 已发布为 apt 包）：

| 环节 | 实现 |
|---|---|
| 世界插件 | `sorting_world.world` 中 `libgazebo_model_attachment_plugin_lib.so` |
| 服务 | `/gazebo/attach`、`/gazebo/detach`（`gazebo_model_attachment_plugin_msgs`） |
| 调用时机 | `motion_executor_node`：**抓取确认成功之后**吸附，放置释放后解除 |
| 吸附点 | 父 `panda/panda_link7`（Gazebo 把固定关节的 `panda_link8`/`panda_hand` 合并进了 link7）|
| 目标识别 | 在夹爪 XY 半径 50 mm 内、前缀为 `sort_obj_` 的模型中取最近者 |

该库的语义（读源码确认）：`attach()` 会把子模型瞬移到父连杆原位并 `AddChild` 成机械臂的子模型，
`detach()` 再摘除关节并归还给世界。因此**物体与手指的接触关系在吸附后改变**，
抬升后的保持判据必须换成"物体是否仍随夹爪移动"。

> 如实说明：这是**仿真层面的简化**（真实系统对应的是摩擦夹持）。抓取确认（闭合瞬间的
> 手指接触判定）、抓取前二次校正、失败重试、放置复核、指标统计等闭环机制全部保留不变。

---

## 4. 视觉方案

### 4.1 两个可切换后端

| 后端 | 说明 | 用途 |
|---|---|---|
| `classical` | HSV 颜色 + 连通域分割，掩码质心/PCA 主轴 | **冷启动**：模型训练完成前即可跑通完整闭环 |
| `yolo` | YOLO11-seg 实例分割（ultralytics） | 正式方案；模型缺失时自动回退到 classical |

启动参数：`vision_backend:=yolo model_path:=/path/to/best.pt`

### 4.2 检测结果清洗（重要）

仅有颜色/分割结果不足以稳定抓取，本项目叠加了三重几何过滤：

1. **面积上下限**（`min_mask_area` / `max_mask_area`）——剔除分拣箱等大色块误检
2. **深度过滤**（`min_object_height`）——三维点低于"桌面+12 mm"判为桌面/地面噪声，整条检测丢弃
3. **工作区 ROI**（`scene.workspace_roi`）——只有落在机械臂工作区内的目标才参与抓取

### 4.3 仿真合成数据 + 域随机化

`synthetic_dataset` 节点自动生成 YOLO-seg 数据集：

1. 调用 `/sorting/reset_scene` 随机重摆（位姿 / 类别 / 颜色 / 光照 / 相机安装抖动）
2. 取一帧 RGB，读取 `/sorting/ground_truth` 真值与 TF 外参
3. 将物体三维几何投影到图像平面得到轮廓多边形
4. 按**深度排序光栅化**做可见性判断，剔除被遮挡部分
5. 输出 YOLO-seg 归一化多边形标注 + `data.yaml` + 预览图

```bash
# 生成 500 帧合成数据集（需先启动仿真与 scene_manager）
ros2 run sorting_vision synthetic_dataset --ros-args \
  -p output_dir:=/mnt/d/llm_training/robot_sorting_project/datasets/sorting_seg \
  -p num_samples:=500

# 训练 YOLO11-seg
yolo segment train data=<数据集>/data.yaml model=yolo11n-seg.pt epochs=100 imgsz=640
```

---

## 5. 实验与指标

`metrics_logger_node` 输出到 `logs/<时间戳>/`：

- `statistics.csv` —— 指标时间序列（状态、尝试数、成功率、准确率、定位误差、周期时间）
- `summary.json` —— 最近一次快照
- `task_status.log` —— 状态机迁移记录

生成报告与图表：

```bash
ros2 run sorting_evaluation make_report --run-dir logs/20260930_181500
# 输出 report.json / report.md / metrics.png
```

指标口径：

| 指标 | 计算方式 |
|---|---|
| 三维定位误差 | 检测三维点 vs 真值物体（平面距离 + 高度差），仅用于评估，不参与控制 |
| 抓取成功率 | 抓取确认成功次数 / 尝试次数 |
| 分拣准确率 | 真值判定落在对应分拣区内的物体数 / 目标总数 |
| 平均周期时间 | 单次 `execute_pick_place` 总耗时均值 |

推荐的对比实验（见 teach.txt 第十五节）：

1. **有无闭环校正对比**：`StartSorting{use_closed_loop: false}` vs `true`
2. **检测框中心 vs 掩码质心**：`pose_estimator_node` 的 `use_mask_centroid` 参数
3. **不同物体密度**：通过 `ResetScene{n_objects}` 调整稀疏/密集场景

---

## 6. 实测验证结果

| 项目 | 结果 |
|---|---|
| 视觉三维定位精度 | 平面误差 **3~7 mm**（蓝圆柱/黄盒/红方块/绿长条混合场景） |
| 视觉误检 | 清洗后 **0 误检**（4 个真值物体 → 恰好 4 条检测） |
| 抓取位姿精度 | `panda_link8` 到位误差 **约 1.5 mm** |
| MoveIt 规划 | 位姿目标与直线插补均正常（无碰撞报错） |
| 闭环校正 | 触发正常，实测校正量可达毫米级 |
| 抓取接触判定 | 稳定检出（手指被挡位置 0.013~0.031，未接触时为 0.000） |
| **搬运保持** | 接入第三方吸附附件后**可完成"抓取→搬运→入箱"全链路**（见 3.5） |
| 抓取成功率 | 实测约 **0.38**（8 次尝试 3 次成功；失败以"夹爪闭合到底"为主） |
| 分拣准确率 | 实测约 **0.50**（真值判定已进入对应分拣区，4 个目标中 2 个） |
| 平均周期时间 | 约 **29 s** / 次抓取放置 |
| 放置视觉复核 | ⚠️ 存在**漏判**：物体实际已入箱，但 `/sorting/targets` 未在箱内检出该目标 |

> 指标来自 `metrics_logger_node` 实测快照（`logs/<时间戳>/summary.json`），
> 抓取成功率与分拣准确率仍受"夹爪闭合到底"这一未解决抓取失败模式影响。

---

## 7. 环境搭建要点（踩坑记录）

这些是在 Windows + WSL2 上跑通本项目必须处理的点，已在代码/脚本中固化：

1. **官方 Panda 描述缺少 `<inertial>`** → sdformat 会丢弃所有 link，Gazebo 中生成空模型。
   解决：`urdf_utils.inject_inertials` 在 xacro 展开后注入 Franka 官方惯量参数
   （数值取自 `franka_description` 的 `inertials.yaml`）。
2. **机械臂底座未固定会自由落体** → URDF 中加入名为 `world` 的 link 与固定关节，
   Gazebo 会把 `world` 识别为世界坐标系并锁死底座。
3. **Gazebo 启动时联网拉取在线模型库会长时间阻塞** → 在 `scripts/env.sh` 中把
   `GAZEBO_MODEL_DATABASE_URI` 指向本地不存在路径，使查询立即失败。
4. **`gazebo_ros2_control` 的 position 接口是运动学瞬移**（`SetPosition(...,true)`），
   且其 `dt` 以纳秒传给 `control_toolbox::Pid`。要获取物理力控必须给出 `pos_kp` 等参数，
   且应只给 P 增益以规避 dt 单位缺陷。
5. **URDF 与 SDF 的 `<box>` 语法不同**：URDF 用 `size` 属性，SDF 用 `<size>` 子元素。
   写错会让整个 collision 被静默丢弃（本项目曾因此导致指尖"穿过"物体抓不住）。
6. **回调阻塞会耗尽 executor 线程池**：在定时器回调里阻塞等待服务，会让同一回调组的
   后续回调把 `MultiThreadedExecutor` 的线程占满，响应回调用无线程可执行、请求永远超时。
   场景装配与任务循环因此都放在独立线程中执行。
7. **第三方包的 `gazebo_media_path` 会顶掉 Gazebo 自己的资源路径**（本次踩坑）：
   `gazebo_model_attachment_plugin` 的 `package.xml` 导出了 `gazebo_ros/gazebo_media_path`，
   而 `gazebo_ros` 的 `gzserver.launch.py` 会据此**整体覆盖** `GAZEBO_RESOURCE_PATH`，
   导致 Gazebo 找不到自带着色器，报
   `[Err] [RTShaderSystem.cc:480] Unable to find shader lib`，
   渲染场景初始化失败、`gzserver` 启动约 2 秒即断言崩溃（表现为相机话题全无、`/spawn_entity` 不可用）。
   解决：在 `scripts/env.sh` 中显式 `export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11`。
8. **Gazebo 会合并固定关节的子连杆**：URDF 里的 `panda_link8` / `panda_hand` 在 Gazebo 模型中
   **并不存在**（被并进 `panda_link7`），可用 `/gazebo/link_states` 核实。
   吸附附件的父连杆因此应填 `panda_link7`。

---

## 8. 目录结构

```
robot_sorting_project/
├── src/
│   ├── sorting_interfaces/          # 自定义消息与服务
│   ├── sorting_bringup/             # 全局配置 + 顶层 launch
│   ├── sorting_simulation/          # 世界 / URDF / 场景管理
│   ├── sorting_vision/              # 检测节点 + 合成数据生成
│   ├── sorting_pose_estimation/     # 三维定位与坐标变换
│   ├── sorting_grasp_planning/      # 抓取位姿生成
│   ├── sorting_motion_control/      # MoveIt 接口 / 夹爪 / 动作执行
│   ├── sorting_task_planning/       # 任务调度与状态机
│   └── sorting_evaluation/          # 指标记录与报告
├── scripts/                         # env.sh / build.sh / run_sim.sh / run_sorting.sh
├── docs/                            # 补充文档
├── logs/                            # 实验日志输出
└── datasets/                        # 合成数据集输出
```