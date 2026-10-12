# 简历写法 · 毕设结构 · 实验设计

> 本文档描述**当前链路**（点云 → GraspNet 6DoF → MoveIt 执行，单一投放点）。
> 早期"2D 检测（YOLO/颜色分割）+ 垂直四段式 + 按类别分拣"链路的描述已作废，见 git 历史。

## 一、简历项目描述

### 项目名称

**基于点云 6DoF 抓取检测与运动规划的机械臂无序抓取系统仿真**

### 偏工程向

- 基于 ROS 2 Humble + MoveIt 2 + Gazebo Sim 搭建机械臂无序抓取（bin picking）仿真平台，
  完成点云感知、6DoF 抓取检测、轨迹规划与抓取执行的完整闭环，代码按 7 个功能包模块化组织。
- 复用预训练 **GraspNet** 实现零训练 6DoF 抓取检测（GPU 推理）；设计点云预处理链
  （工作区 ROI + 高度夹层 + RANSAC 去平面 + DBSCAN 欧式聚类）把相机点云裁剪为工作区物体簇点，
  解决合成场景"桌面点占比 ~99% 导致抓取网络采样失效"的问题。
- 推导 GraspNet 抓取姿态到 Franka 末端（`panda_link8`）的解析映射
  （`approach→+z`、`closing→` 指爪闭合轴），垂直下抓时精确退化为经典姿态；
  调用 MoveIt 2 完成位姿规划与碰撞检测，下降/抬升/投放段采用直线插补（`compute_cartesian_path`）。
- 实现 effort 力控夹爪（张合/保持分阶段力矩）与基于手指关节位置的抓取确认（真实接触物理），
  建立"闭合确认 + 抬升滑落检测 + 多候选重试"的鲁棒执行流程。
- 工程化落地：按节点注入 `.venv` 依赖（避免 venv 的 numpy 1.26 破坏 MoveIt）；
  分层启动（仿真/MoveIt 常驻、业务节点快速重启）；指标记录与报告生成。

### 偏算法向

- 复用预训练 GraspNet 完成 6DoF 抓取检测，规避从零训练与数据集标注；针对合成场景设计
  "ROI + 高度夹层 + 平面剔除 + 聚类"预处理链，并只向网络输入筛选后的物体簇点云（而非含杂点的全量点）。
- 推导 GraspNet 姿态约定（`R[:,0]=approach`、`R[:,1]=closing`）并解析构造末端旋转矩阵，
  以"垂直下抓退化一致性"作为可回归的正确性校验。
- 抓取候选过滤与累积：夹爪开口过滤（78 mm 上限）、接近方向过滤（面向下抓约束）、
  时序累积 + 体素去重（弥补静态场景下单帧覆盖不全）。
- 设计基于接触状态（手指关节位置）的抓取成功判据与抬升后滑落检测，失败自动切换下一候选。

---

## 二、毕设论文结构建议

**题目：基于点云 6DoF 抓取检测与运动规划的机械臂无序抓取系统设计与实现**

- **第 1 章 绪论**：研究背景、国内外现状（bin-picking / 6DoF 抓取检测 / 抓取网络）、研究内容与章节安排
- **第 2 章 系统总体方案设计**：需求分析、系统架构（7 个功能包与模块映射）、
  软硬件平台选型（ROS 2 / MoveIt 2 / Gazebo Sim / Panda / RGB-D）、无序抓取任务定义
- **第 3 章 点云感知与工作区提取**：
  - 坐标系统一（相机光学系 / world 系）
  - 工作区 ROI 与高度夹层裁剪、RANSAC 平面剔除、DBSCAN 欧式聚类
  - 只发布物体簇点云对抓取网络输入质量的影响
- **第 4 章 6DoF 抓取检测与末端映射**：
  - 复用预训练 GraspNet 的零训练路线与网络输入/输出约定
  - GraspNet 姿态（approach / closing）到 Panda 末端（`panda_link8`）的解析映射与回归验证
  - 抓取候选过滤（开口 / 接近方向）与时序累积、体素去重
- **第 5 章 运动规划与抓取执行**：
  - Panda 运动学建模要点（惯量注入、底座固定、ros2_control 真实力矩驱动）
  - MoveIt 位姿规划与直线插补的适用场景划分
  - effort 力控夹爪的分阶段力矩策略
  - 基于接触状态的抓取确认与抬升滑落检测
- **第 6 章 实验与结果分析**：
  - 抓取成功率实验（多候选重试 / 接近方向阈值 / 场景密度）
  - 规划成功率与周期时间
  - Gazebo Sim 接触仿真保真度对搬运保持的影响分析（诚实讨论）
- **第 7 章 总结与展望**：不足（单一投放点、无分类、Sim2Real）、未来工作
  （点云层面分类、6D 位姿估计、堆叠场景、传送带动态分拣）

---

## 三、对比实验设计

### 实验 1：接近方向阈值 `max_approach_deg`

GraspNet 训练于侧视堆叠场景，会提出大量侧向抓取。收紧阈值（如 30°）应提升规划成功率，
放宽（如 75°）可能提升候选覆盖率但增加不可达/奇异位形。

```bash
# 编辑 sorting_bringup/config/system.yaml 的 graspnet.max_approach_deg 后重启业务节点
ros2 launch sorting_bringup sorting_nodes.launch.py
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}"
```

对比指标：抓取成功率、平均周期时间、失败模式分布。

### 实验 2：时序累积窗口 `accumulate_window`

静态场景下 GraspNet 单帧只对部分物体提出抓取，累积窗口用于补齐覆盖：

```bash
# accumulate_window: 0（关闭）vs 3.0（开启）
```

对比指标：单轮可抓物体覆盖数、抓取成功率、候选数量。

### 实验 3：场景密度（遮挡）影响

```bash
# 稀疏
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 3, randomize_class: true, randomize_pose: true, randomize_scene: false}"
# 密集
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 7, randomize_class: true, randomize_pose: true, randomize_scene: false}"
```

对比指标：抓取成功率、平均周期时间。`scene.scatter.min_separation` 控制最小间距，
密集场景可在配置中调小。

---

## 四、可强调的创新点

1. **零训练 6DoF 抓取**：复用预训练 GraspNet，免去数据集标注与训练调参；
   面向"末端下抓"的实际约束施加接近方向过滤。
2. **面向"桌面主导"场景的点云预处理链**：ROI + 高度夹层 + 平面剔除 + 聚类，
   并只向抓取网络输入物体簇点云。
3. **GraspNet → Panda 末端姿态的解析映射**：以"垂直下抓退化一致性"作为可回归校验，
   避免姿态映射搞反导致夹爪从错误方向下探。
4. **诚实的接触物理判据**：基于手指关节位置识别"未夹住 / 抬升滑落"两类失败并自动切换候选。
5. **工程化环境治理**：按节点注入 `.venv` 依赖（隔离 venv numpy 与 MoveIt 系统 numpy）、
   分层启动与可复现构建。