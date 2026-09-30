# 仿真场景中单个物体的真值状态（用于合成数据标注与评估对比）
std_msgs/Header header

uint8 class_id
string class_name
string object_name               # Gazebo 模型名
uint8 shape                      # 0=box 1=cylinder 2=sphere

geometry_msgs/Pose pose          # world 坐标系下的位姿
geometry_msgs/Vector3 size       # 包围盒尺寸 (x, y, z) 米

bool is_grasped                  # 是否已被机械臂抓取
string placement_zone            # 应放置的分拣区