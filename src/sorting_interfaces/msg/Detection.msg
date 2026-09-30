# 单次视觉检测 / 分割结果
std_msgs/Header header

uint8 class_id
string class_name
float32 confidence

# ---- 2D 检测框（像素坐标，左上 / 右下）----
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# ---- 分割掩码信息 ----
bool has_mask
float32 centroid_u          # 掩码质心 u（像素）
float32 centroid_v          # 掩码质心 v（像素）
float32 mask_area           # 掩码面积（像素）
float32 yaw_image           # 掩码主轴方向，相对图像 u 轴（弧度）
float32 elongation          # 主轴/次轴特征值比（判断是否长条物体）

# ---- 三维定位结果 ----
bool has_3d
geometry_msgs/Point position_camera   # 相机光学坐标系下的位置
geometry_msgs/Point position_base     # 机械臂基坐标系（panda_link0）下的位置
float32 yaw_base                      # 物体主轴在基坐标系 XY 平面内的方向（弧度）
float32 depth_stddev                  # 深度邻域标准差（深度噪声指标）