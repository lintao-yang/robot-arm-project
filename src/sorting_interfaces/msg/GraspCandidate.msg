# 抓取候选：四段式抓取策略（pre-grasp / grasp / lift / place）
std_msgs/Header header

uint8 class_id
string class_name
float32 confidence

geometry_msgs/Pose pregrasp_pose   # 目标上方安全高度
geometry_msgs/Pose grasp_pose      # 实际抓取位姿
geometry_msgs/Pose lift_pose       # 抓取后抬升位姿
geometry_msgs/Pose place_pose      # 分类放置位姿

float32 score                      # 抓取质量评分
string placement_zone              # 目标放置区名称