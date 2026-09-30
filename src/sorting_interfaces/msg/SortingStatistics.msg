# 系统级分拣统计指标
std_msgs/Header header

# ---- 任务计数 ----
uint32 total_targets        # 场景中待分拣目标总数
uint32 attempted            # 已尝试抓取次数
uint32 picked_success       # 抓取成功次数
uint32 placed_correct       # 正确放入对应分拣区的次数
uint32 grasp_failures       # 抓取失败次数
uint32 retries              # 重试次数
uint32 closed_loop_corrections  # 触发二次视觉校正的次数

# ---- 指标 ----
float32 grasp_success_rate      # 抓取成功率 = picked_success / attempted
float32 sorting_accuracy        # 分拣准确率 = placed_correct / total_targets
float32 avg_cycle_time          # 平均单次抓取周期（秒）
float32 total_time              # 任务总耗时（秒）

# ---- 定位精度（与仿真真值对比）----
float32 mean_position_error_mm  # 平均三维定位误差（毫米）

# ---- 当前状态 ----
uint32 current_target_index
string current_class
string state                    # 状态机当前状态