"""sorting_grasp_planning：模块4 抓取决策（GraspNet 6DoF 后端）。

不做急切导入：graspnet_planner_node 依赖 torch 等重依赖，
急切导入会让任何 import sorting_grasp_planning 的节点都被迫加载整条依赖链。
"""

__all__ = []