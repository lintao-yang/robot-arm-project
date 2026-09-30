# 简历写法 · 毕设结构 · 实验设计

## 一、简历项目描述

### 项目名称

**基于 ROS 2 与深度视觉的机械臂闭环分拣仿真系统**

### 偏工程向

- 基于 ROS 2 Humble + MoveIt 2 + Gazebo 搭建机械臂视觉分拣仿真平台，完成相机感知、
  目标三维定位、轨迹规划与分类放置的完整闭环流程，代码按 9 个功能包模块化组织。
- 使用实例分割（YOLO11-seg / 颜色分割双后端）提取目标掩码、质心与主轴方向，
  结合 RGB-D 深度图与 TF2 外参变换实现像素坐标到机械臂基坐标系的三维定位，
  实测平面定位误差 3~5 mm。
- 设计 pre-grasp / grasp / lift / place 四段式抓取策略，抓取高度由"手掌避碰 + 指尖不扎桌"
  双几何约束自适应求解；调用 MoveIt 2 完成位姿规划与碰撞检测，接近/抬升/放置段采用
  直线插补（`compute_cartesian_path`），避免非直线路径拖拽目标。
- 实现 effort 力控夹爪（张合力矩实测标定）与基于手指关节位置的抓取确认机制，
  建立"抓取前二次视觉校正 + 抓取失败重试 + 放置后视觉复核"的闭环反馈，
  并用明确状态机（IDLE→DETECT→SELECT→PLAN_PICK→EXECUTE→VERIFY→CHECK→DONE/RETRY）组织任务流。
- 搭建仿真合成数据自动生成流水线（几何投影标注 + 深度排序遮挡剔除 + 域随机化），
  配套指标记录与实验报告生成，统计抓取成功率、分拣准确率、定位误差与周期时间。

### 偏算法向

- 设计并实现基于实例分割与深度恢复的机械臂视觉抓取系统，利用掩码质心与 PCA 主轴
  提取目标位姿信息，并针对 RGB-D 深度噪声采用窗口内中位数滤波与深度一致性校验。
- 建立从像素坐标 → 相机光学系 → 机械臂基座系的完整坐标变换链，
  以相机安装位姿推导光学外参并构建统一 TF 树，实现感知与执行的坐标统一。
- 提出抓取前局部重识别与误差补偿策略：在预抓取位对目标进行二次检测与重规划，
  抑制远距离观测误差对抓取精度的影响。
- 设计基于接触状态（手指关节位置）的抓取成功判据与抬升后滑落检测，
  实现失败自动重试与目标跳过，提升多目标场景下的系统鲁棒性。

---

## 二、毕设论文结构建议

**题目：基于深度视觉与运动规划的机械臂闭环分拣系统设计与实现**

- **第 1 章 绪论**：研究背景、国内外现状（视觉抓取 / bin-picking / 分拣自动化）、研究内容与章节安排
- **第 2 章 系统总体方案设计**：需求分析、系统架构（9 个功能包与 7 个模块映射）、
  软硬件平台选型（ROS 2 / MoveIt 2 / Gazebo / Panda / RGB-D）、分拣任务定义
- **第 3 章 视觉感知与目标定位**：
  - 颜色分割与实例分割两条技术路线对比
  - 仿真合成数据生成方法与域随机化策略
  - 掩码质心与主轴方向提取、深度中位数滤波
  - 像素→相机系→基座系坐标变换与 TF 树设计
  - 检测结果三重几何过滤（面积 / 深度 / 工作区 ROI）
- **第 4 章 机械臂抓取与运动规划**：
  - Panda 运动学模型与 URDF 建模要点（惯量注入、底座固定、ros2_control 接口选择）
  - 四段式抓取位姿生成与"手掌/指尖"双约束夹取高度求解
  - 夹爪闭合轴与末端姿态的映射关系推导（yaw 偏置 45°）
  - MoveIt 2 位姿规划与直线插补的适用场景划分
- **第 5 章 闭环反馈与系统实现**：
  - 状态机设计
  - 抓取前二次视觉校正
  - 基于接触状态的抓取确认与滑落检测
  - 放置后视觉复核
  - ROS 2 节点通信、服务/话题/动作设计与并发模型（阻塞回调与 executor 线程池）
- **第 6 章 实验与结果分析**：
  - 视觉检测与三维定位精度实验
  - 抓取位姿精度实验（TF 实测）
  - 分拣性能实验（成功率 / 准确率 / 周期时间）
  - 有无闭环校正对比实验
  - Gazebo Classic 接触仿真保真度对搬运保持的影响分析（诚实讨论）
- **第 7 章 总结与展望**：不足（搬运保持、Sim2Real）、未来工作（迁移 Gazebo Sim/Isaac Sim、
  6D 姿态估计、抓取网络、传送带动态分拣）

---

## 三、对比实验设计

### 实验 1：有无抓取前二次视觉校正

```bash
# 关闭闭环校正跑一组
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}"
# 记录 logs/<时间戳>/summary.json 后，重置场景再跑开启闭环的一组
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 4, randomize_class: true, randomize_pose: true, randomize_scene: true}"
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: true, continuous: true}"
```

对比指标：抓取成功率、平均三维定位误差、平均周期时间。

### 实验 2：检测框中心 vs 掩码质心

`pose_estimator_node` 参数 `use_mask_centroid`（true = 掩码质心，false = 检测框中心），
对比定位误差与抓取成功率。颜色分割后端本身输出掩码，两者差异主要体现在
非对称/长条目标上。

### 实验 3：场景密度（遮挡）影响

```bash
# 稀疏
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 3, randomize_class: true, randomize_pose: true, randomize_scene: false}"
# 密集
ros2 service call /sorting/reset_scene sorting_interfaces/srv/ResetScene \
  "{n_objects: 7, randomize_class: true, randomize_pose: true, randomize_scene: false}"
```

对比指标：漏检率、定位误差、抓取成功率、平均周期时间。
注意 `scene.scatter.min_separation` 控制最小间距，密集场景可在配置中调小。

---

## 四、可强调的创新点

1. **抓取前二次视觉校正 + 抓取/放置双向复核**的三层闭环反馈机制
2. **手掌/指尖双几何约束的夹取高度自适应求解**（避免手掌压物与指尖扎桌）
3. **检测结果三重几何过滤**（面积 / 深度 / 工作区 ROI）显著抑制仿真场景下的误检
4. **仿真合成数据自动生成 + 深度排序遮挡剔除**的数据流水线
5. **颜色分割与实例分割双后端可切换**，实现"先打通闭环再升级模型"的渐进式工程路线
6. **诚实的失败检测机制**：能稳定识别"未夹住"与"抬升滑落"两类失败并触发重试