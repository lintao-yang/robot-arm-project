# 已知问题与后续改进方向

## 1. 搬运阶段物体保持不稳定（Gazebo Classic 接触仿真）

### 现象

抓取动作链本身可以跑通，且**抓取接触判定稳定有效**：

```
夹爪闭合: 手指位置=0.0188 阈值=0.006 => 夹住      # 指尖确实被物体挡住
抬升后手指位置 = -0.0 (闭合时 0.0188)             # 抬升过程中物体脱出
抓取失败：抬升后检测到物体滑落
```

即：**能夹住，但搬不动**。当前在 Gazebo Classic 11 下搬运成功率很低。

### 已排查并排除的原因

| 假设 | 排查方式 | 结论 |
|---|---|---|
| 视觉定位不准 | 与真值对比 | 平面误差仅 3~5 mm，**排除** |
| 抓取位姿算错 | TF 实测 `panda_link8` 到位误差 1.5 mm；手指关节原点与物体位置吻合 | **排除** |
| 夹爪力矩方向错 | 直接下发 ±8 N，实测 +8 N 张开 / -8 N 闭合 | **排除** |
| 指尖碰撞体缺失 | 转 SDF 后发现 URDF 的 `<box>` 写成了 SDF 语法，导致 collision 被丢 | **已修复**（修复后可检出接触） |
| 接触修正速度受限 | 移除 `<max_vel>` / `<min_depth>`，加硬 `kp=1e6` | 无改善 |
| 摩擦不足 | 手指 μ 提到 20、物体 μ 提到 5 | 无改善 |
| 抬升路径绕行 | 改用 `/compute_cartesian_path` 直线抬升 | 无改善 |
| mimic 关节冲突 | 从 URDF 删除 `panda_finger_joint2` 的 `<mimic>` | 无改善 |
| 夹取高度导致手掌压物 | 改为"手掌约束 + 指尖约束"自适应高度 | 无改善（但该修复本身是正确且有价值的） |
| 机械臂改为力控位置伺服 | 给关节加 `pos_kp=1500` 切到 `SetForce` 分支 | **更差**（稳态下垂导致规划失败增多），已回退 |

### 根因分析（当前最优解释）

`gazebo_ros2_control` 0.4.x 对**普通 `position` 命令接口**调用
`Joint::SetPosition(pos, true)`，即**运动学瞬移**。这带来两个后果：

1. 机械臂没有真实动力学，运动不会被物体阻挡，也不会把接触力反传回基座；
2. 手指虽然走 `effort` 力控分支（真实受力），但它挂在一个**运动学驱动**的链上。
   当末端被瞬移上抬时，被夹物体与手指之间只靠"接触摩擦"耦合，而这种
   "刚体瞬移体 + 接触约束"的组合在 ODE 求解器下本身就很容易解耦——表现为物体滑脱。

社区对 Gazebo Classic 下机械臂抓取搬运的通用做法是使用
[`gazebo_ros_link_attacher`](https://github.com/pal-robotics/gazebo_ros_link_attacher)
在夹爪闭合后创建固定关节（"吸附"），这恰恰说明该问题在 Gazebo Classic 中普遍存在。

### 已实施的修复（本次）

已按下面第 2 条路径接入**现成第三方库**
[`gazebo_model_attachment_plugin`](https://github.com/Boeing/gazebo_model_attachment_plugin)
（apt：`ros-humble-gazebo-model-attachment-plugin`），**没有自研任何物理实现**：

- 世界文件加载 `libgazebo_model_attachment_plugin_lib.so`，提供 `/gazebo/attach`、`/gazebo/detach`；
- `motion_executor_node` 在**抓取确认成功之后**调用 attach，放置释放后调用 detach；
- 父连杆取 `panda/panda_link7`——Gazebo 会把固定关节的 `panda_link8`/`panda_hand` 合并进 link7，
  这一点必须用 `/gazebo/link_states` 核实，否则会报 `Could not find link panda_hand on model panda`。

**该库的语义**（读源码确认）：`attach()` 会把子模型 `SetWorldPose` 到父连杆原位并
`m1->AddChild(m2)`，使物体成为机械臂的子模型；`detach()` 摘除关节并归还世界。
由此带来一个连锁影响：**吸附后物体与手指的接触关系改变**，原先"抬升后读手指位置判断是否滑落"
会误报"滑落"。因此把已吸附后的保持判据改为"物体是否仍随夹爪移动"
（`motion_executor_node._check_retention`，比较 `/gazebo/model_states` 中物体与抓取点的水平距离）；
**闭合瞬间的接触判定仍是纯物理量**，闭环性质不变。

**修复后的实测**（`gui:=true`，4 个物体，闭环校正开启）：

```
attempted: 8   picked_success: 3   placed_correct: 2
grasp_success_rate: 0.375   sorting_accuracy: 0.5   mean_position_error_mm: 6.5
```

物体真值显示确实落进了对应分拣箱（如 `sort_obj_2` 在 zone_d 箱内 x=0.660/y=0.270，z=0.0405）。

### 仍未解决的问题

| 问题 | 现象 | 当前判断 |
|---|---|---|
| 抓取成功率偏低（约 0.38） | 失败集中在"夹爪闭合到底，未夹住物体"，即下爪时物体不在两指之间 | 与下爪位姿/物体是否被前一次失败动作碰偏有关，尚未定位 |
| 放置视觉复核漏判 | 物体**已确实进入分拣箱**（真值确认），但 `_verify_placement` 报 False | `/sorting/targets` 未在箱内检出该目标（疑为箱体颜色与物体相近，颜色分割把箱壁与物体并成一个大色块后被面积上限剔除），待确认 |
| 吸附服务偶发超时 | 第二次 `attach` 调用超时后 `gzserver` 挂死（`/gazebo/model_states`、`/clock` 停发） | 该库的 attach 会 `SetPaused(true)` + 持物理更新互斥锁；与运动学驱动机械臂组合时的稳定性需进一步验证 |

1. **迁移到 Gazebo Sim（Ignition）/ Isaac Sim**
   新仿真器对接触与关节约束的求解质量明显更好，是首选方案。
   本项目除 `sorting_simulation` 之外的 8 个包**完全不需要改动**，
   只需重写世界文件与 ros2_control 硬件插件配置。

2. **在 Gazebo Classic 中引入抓取吸附附件**
   加入 `gazebo_ros_link_attacher`：夹爪闭合确认成功后，调用服务在
   `panda_hand` 与目标模型之间建立固定关节；释放时拆除。
   需要在 README/论文中**如实说明**这是仿真层面的简化（真实系统里对应的是摩擦夹持）。
   好处是闭环逻辑（抓取确认、放置复核、失败重试、指标统计）全部保留不变。

3. **继续调参**（性价比低）
   可尝试：把物体质量调到 0.01~0.02 kg、关闭物体与桌面的接触阻尼、
   把 `max_step_size` 降到 0.0005、单独给手指关节加 `<damping>`。

### 值得强调的一点

尽管搬运保持未解决，**闭环反馈机制本身是正确且有效的**——系统能够
自行检出"未夹住"与"抬升后滑落"两种失败，并进入 RETRY / 跳过该目标，
这正是 `teach.txt` 所要求的闭环价值所在。评估指标会如实反映当前成功率。

---

## 2. 其他注意事项

- **Gazebo 图形界面**：`gui:=true` 会通过 WSLg 打开 gzclient 窗口；无界面运行时
  相机仍依赖离屏 OpenGL 渲染，因此 `scripts/env.sh` 中设置了 `DISPLAY=:0`。
- **性能**：仿真物理 500 Hz（步长 2 ms）、相机 10 Hz 960×720。若机器性能不足，
  可降低相机分辨率或 `<update_rate>`，否则 `/spawn_entity` 等服务可能响应缓慢。
- **RViz2 中的 TF**：`world → panda_link0` 由 `robot_state_publisher` 发布；
  先启动 `move_group.launch.py rviz:=true` 再启动视觉节点即可看到完整链路。
- **首次运行时** `scene_manager` 需要若干秒装配场景；若 Gazebo 服务拥塞导致失败，
  会打印告警，可手动调用 `/sorting/reset_scene` 重新摆放。