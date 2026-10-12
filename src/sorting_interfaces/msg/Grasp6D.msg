# 6DoF 抓取候选（来自抓取检测网络）
std_msgs/Header header

geometry_msgs/Pose pose          # 抓取位姿（world 系）；姿态的 z 轴为夹爪接近方向
geometry_msgs/Vector3 approach   # 接近方向单位向量（world 系）
float32 width                    # 所需夹爪开口（米）
float32 score                    # 抓取质量评分（越高越好）