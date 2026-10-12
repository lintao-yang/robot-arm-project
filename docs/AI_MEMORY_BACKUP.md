# AI 协作上下文备份（关键决策沉淀）

> 用途：跨会话/跨环境时，保证新会话能靠读文件重建项目认知。
> 数据来源：项目 memory（硬约束 / 工程约定 / 经验教训）与历次对话主题摘要。

## 1. 硬约束（Hard Constraints）

- 运行环境：Ubuntu 22.04 + ROS 2 Humble，源码位于 ext4 原生分区（勿放 NTFS 挂载盘）
- 启动项目前必须 `source scripts/env.sh` 载入环境变量
- 仿真后端：Gazebo Sim (Fortress)，非 Gazebo Classic
- 必须设置 `GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share`，否则机器人模型缺失网格与碰撞几何
- 需把 `/world/sorting_world/clock` 桥接重映射至 `/clock`，否则 `use_sim_time` 节点定时器冻结
- 项目 Python 依赖（torch / scikit-learn）装在 `<workspace>/.venv`，**按节点注入**到需要它的
  节点（见 `sorting_bringup/venv_env.py`）；**禁止**全局注入 venv 的 site-packages
  —— venv 的 numpy 1.26 已移除 `np.float`，会直接让 MoveIt 启动失败
- GPU 为主要计算资源（GraspNet 推理用 `cuda:0`）

## 2. 工程约定（Conventions）

- 7 个功能包模块化组织：`sorting_interfaces` / `sorting_bringup` / `sorting_simulation` /
  `sorting_perception` / `sorting_grasp_planning` / `sorting_motion_control` / `sorting_evaluation`
- 启动脚本为 `.sh`，位于 `scripts/`，支持 `gui:=true/false` 参数
- 系统服务接口：`/sorting/start_sorting`、`/sorting/reset_scene`
- 类别配置唯一真值源为 `classes.yaml`（当前仅用于仿真物体尺寸/颜色与分拣箱位姿）
- ext4 原生分区启用 `--symlink-install`：改 `.py` 后无需重新 `colcon build`

## 3. 关键决策与教训（Lessons）

- **方案 A（当前唯一链路）**：点云 → 预处理 → GraspNet 6DoF → MoveIt 执行。
  已**整体移除** 2D 检测（YOLO/颜色分割）、3D 定位、垂直四段式抓取、按类别分拣的旧链路
  （相关功能包 `sorting_vision` / `sorting_pose_estimation` / `sorting_task_planning` 及
  `grasp_planner_node` / `motion_executor_node` 已删除，接口 `Detection` / `GraspCandidate` /
  `PlanGrasp` / `ExecutePickPlace` 已删除）。
- **投放策略**：GraspNet 只给抓取姿态、不带类别，故不恢复分类；所有物体投放至**单一投放点**
  （`graspnet.drop_pose`）。分类需求留待后续在点云层面迭代，不引入 YOLO。
- **GraspNet 姿态约定**：`R[:,0]=approach`、`R[:,1]=closing`、`R[:,2]=height`
  （来源：`third_party/graspnet-baseline/utils/loss_utils.py` 与 graspnetAPI 指爪几何）。
- **末端映射**：`panda_link8` 的 +z 为接近方向、闭合轴在 link8 系为 `normalize(1,1,0)`；
  该构造在垂直下抓时精确退化为 `Rz(45°)·diag(1,-1,-1)`，可作回归校验。
- **world 系 == panda_link0 系**：机器人以 `(0,0,0)`、无旋转生成，抓取位姿可直接喂 MoveIt。
- **主选逆运动学**：不使用已弃用的 pick_ik。
- 抓取失败集中于**接触物理**（滑落/楔开）而非指令通路；`gripper` 保留 0.5s 周期保持力矩刷新作保险。

## 4. 当前进度快照

- P0 点云预处理、P1 GraspNet 6DoF 检测、**P2 GraspNet→MoveIt 抓取执行已打通并实测成功**
  （在运行中的仿真里触发单次抓取，MoveIt 六段轨迹全部成功，抓取确认基于真实接触）。
- 环境治理：`venv_env.py` 按节点注入 venv 依赖；`sorting_system.launch.py` 一键起全链路。
- 待办：抓取成功率实测（多候选重试 / 接近方向阈值 / 场景密度）→ 结果沉淀 → 文档同步 → git 提交。

## 5. 换环境后如何续接

1. 读本文件 + `README.md` + `docs/KNOWN_ISSUES.md` 重建认知
2. 按 `docs/UBUNTU_NATIVE_MIGRATION.md` 完成环境重建（并确认 `.venv` 存在）
3. 从「抓取成功率实测」继续推进