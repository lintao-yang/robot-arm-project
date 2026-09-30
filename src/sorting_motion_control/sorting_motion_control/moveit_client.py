"""MoveIt2 运动接口封装。

直接使用 ``/move_action``（moveit_msgs/action/MoveGroup）这一官方 Action 接口，
不依赖任何第三方 Python 绑定，规划与执行均由 move_group 完成（含碰撞检测）。
"""

import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

# MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1


class MoveGroupClient:
    """对 move_group 的位姿 / 关节空间运动请求封装（同步阻塞式）。"""

    def __init__(
        self,
        node,
        group_name: str = 'panda_arm',
        ee_link: str = 'panda_link8',
        base_frame: str = 'panda_link0',
        joint_names=None,
        planner_id: str = 'RRTConnect',
        position_tolerance: float = 0.006,
        orientation_tolerance: float = 0.08,
        planning_time: float = 5.0,
        planning_attempts: int = 5,
        action_name: str = '/move_action',
        cartesian_service: str = '/compute_cartesian_path',
        execute_action: str = '/execute_trajectory',
    ):
        self._node = node
        self.group_name = group_name
        self.ee_link = ee_link
        self.base_frame = base_frame
        self.joint_names = list(joint_names or [])
        self.planner_id = planner_id
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts

        self._client = ActionClient(node, MoveGroup, action_name)
        self._execute_client = ActionClient(node, ExecuteTrajectory, execute_action)
        self._cartesian_client = node.create_client(GetCartesianPath, cartesian_service)
        self._server_ready = False

    # ------------------------------------------------------------------
    def wait_for_server(self, timeout: float = 30.0) -> bool:
        if self._server_ready:
            return True
        if not self._client.wait_for_server(timeout_sec=timeout):
            return False
        self._server_ready = True
        return True

    def _wait(self, future, timeout: float) -> bool:
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        return future.done()

    # ------------------------------------------------------------------
    def _pose_constraints(self, pose: Pose, position_tolerance: float,
                          orientation_tolerance: float) -> Constraints:
        constraints = Constraints()

        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(position_tolerance)]
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(pose)
        position.constraint_region = region
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = self.ee_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = float(orientation_tolerance)
        orientation.absolute_y_axis_tolerance = float(orientation_tolerance)
        orientation.absolute_z_axis_tolerance = float(orientation_tolerance)
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _joint_constraints(self, values) -> Constraints:
        constraints = Constraints()
        for name, value in zip(self.joint_names, values):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _execute(self, request: MotionPlanRequest, timeout: float):
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        send_future = self._client.send_goal_async(goal)
        if not self._wait(send_future, 10.0):
            return False, '发送运动目标超时'
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, 'move_group 拒绝该运动目标'

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, timeout):
            return False, f'运动执行超时（>{timeout:.0f}s）'

        wrapped = result_future.result()
        if wrapped is None:
            return False, '未收到运动结果'
        code = wrapped.result.error_code.val
        if code == MOVEIT_SUCCESS:
            return True, 'ok'
        return False, f'MoveIt 错误码 {code}'

    # ------------------------------------------------------------------
    def move_to_pose(self, pose: Pose, velocity: float = 0.2, acceleration: float = 0.2,
                     position_tolerance: float = None,
                     orientation_tolerance: float = None,
                     timeout: float = 25.0):
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = float(velocity)
        request.max_acceleration_scaling_factor = float(acceleration)
        if self.planner_id:
            request.planner_id = self.planner_id
        request.goal_constraints.append(
            self._pose_constraints(
                pose,
                position_tolerance if position_tolerance is not None else self.position_tolerance,
                orientation_tolerance if orientation_tolerance is not None else self.orientation_tolerance,
            )
        )
        return self._execute(request, timeout)

    def move_to_joint_values(self, values, velocity: float = 0.25,
                             acceleration: float = 0.25, timeout: float = 25.0):
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = float(velocity)
        request.max_acceleration_scaling_factor = float(acceleration)
        if self.planner_id:
            request.planner_id = self.planner_id
        request.goal_constraints.append(self._joint_constraints(values))
        return self._execute(request, timeout)

    # ------------------------------------------------------------------
    # 直线插补（Cartesian）运动
    # ------------------------------------------------------------------
    @staticmethod
    def _retime(trajectory, time_scale: float):
        """按比例拉长轨迹时间轴，用于降低笛卡尔路径的执行速度。"""
        if time_scale <= 0.0:
            return trajectory
        for point in trajectory.joint_trajectory.points:
            seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            seconds *= time_scale
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
            point.velocities = [v / time_scale for v in point.velocities]
            point.accelerations = [a / (time_scale * time_scale) for a in point.accelerations]
        return trajectory

    def move_cartesian(self, waypoints, velocity: float = 0.15, max_step: float = 0.004,
                       min_fraction: float = 0.9, timeout: float = 40.0):
        """沿直线插补运动到给定路点（用于下降抓取、垂直抬升与放置）。

        相比 OMPL 的位姿目标，直线运动不会产生"先横移再抬升"之类的绕行路径，
        是抓取/放置阶段避免把物体拖掉或撞飞的关键。
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            return False, 'compute_cartesian_path 服务不可用'
        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            return False, 'execute_trajectory 服务不可用'

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = list(waypoints)
        request.max_step = float(max_step)
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        if not self._wait(future, 15.0):
            return False, '笛卡尔路径计算超时'
        response = future.result()
        if response is None:
            return False, '笛卡尔路径计算无响应'
        if response.fraction < min_fraction:
            return False, f'直线路径仅规划出 {response.fraction * 100:.0f}%'

        trajectory = self._retime(response.solution, 1.0 / max(velocity, 1e-3))

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        if not self._wait(send_future, 10.0):
            return False, '发送轨迹执行目标超时'
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, '轨迹执行目标被拒绝'

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, timeout):
            return False, '轨迹执行超时'
        wrapped = result_future.result()
        if wrapped is None:
            return False, '未收到轨迹执行结果'
        code = wrapped.result.error_code.val
        if code == MOVEIT_SUCCESS:
            return True, f'ok (fraction={response.fraction:.2f})'
        return False, f'轨迹执行 MoveIt 错误码 {code}'