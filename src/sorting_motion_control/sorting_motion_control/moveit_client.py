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
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
    GetPlanningScene,
    GetStateValidity,
)
from moveit_msgs.msg import PositionIKRequest
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

# MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1

# MoveItErrorCodes 常见码的可读名（取自 moveit_msgs/msg/MoveItErrorCodes.msg），
# 仅用于诊断日志，避免日志里只剩一个孤立数字。
MOVEIT_ERROR_NAMES = {
    1: 'SUCCESS',
    99999: 'FAILURE(通用)',
    -1: 'PLANNING_FAILED',
    -2: 'INVALID_MOTION_PLAN',
    -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    -4: 'CONTROL_FAILED',
    -5: 'UNABLE_TO_ACHIEVE_PLANNING_GOAL',
    -6: 'TIMED_OUT',
    -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION',
    -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12: 'GOAL_IN_COLLISION',
    -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED',
    -15: 'INVALID_GROUP_NAME',
    -16: 'INVALID_GOAL_CONSTRAINTS',
    -17: 'INVALID_ROBOT_STATE',
    -18: 'INVALID_LINK_NAME',
    -19: 'INVALID_OBJECT_NAME',
    -21: 'FRAME_TRANSFORM_FAILURE',
    -22: 'COLLISION_CHECKING_UNAVAILABLE',
    -23: 'ROBOT_STATE_STALE',
    -24: 'SENSOR_INFO_STALE',
    -25: 'COMMUNICATION_FAILURE',
    -31: 'NO_IK_SOLUTION',
}


def moveit_error_name(code: int) -> str:
    """把 MoveIt 错误码转成 ``码(名称)``；未收录的码只回显数字。"""
    return f'{code}({MOVEIT_ERROR_NAMES[code]})' if code in MOVEIT_ERROR_NAMES else str(code)


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
        position_tolerance: float = 0.010,
        orientation_tolerance: float = 0.12,
        planning_time: float = 3.0,
        planning_attempts: int = 10,
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
        self._validity_client = node.create_client(GetStateValidity, '/check_state_validity')
        self._fk_client = node.create_client(GetPositionFK, '/compute_fk')
        self._ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self._scene_get_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        self._server_ready = False

        # ---- 规划请求统计（供评估层计算"规划成功率"，只计数不改行为）----
        # 每次 move_to_pose / move_to_joint_values / move_cartesian 记为一次规划请求；
        # 返回成功即计入成功数。仅统计"规划+执行"类请求，诊断类探针不计入。
        self.plan_requests = 0
        self.plan_successes = 0

    def _record_plan(self, ok: bool):
        """记录一次规划请求及其结果（仅统计）。"""
        self.plan_requests += 1
        if ok:
            self.plan_successes += 1

    def plan_stats(self) -> dict:
        """返回累计的规划请求/成功计数。"""
        return {'plan_requests': self.plan_requests, 'plan_successes': self.plan_successes}

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

    def _abort_and_drain(self, goal_handle, result_future, drain_timeout: float = 40.0):
        """客户端超时放弃前，显式取消目标并等控制器真正停下。

        为什么必须做：`_wait` 超时只是"不再等"，控制器仍在执行那条旧轨迹。此时再发
        新目标会被 move_group 直接拒绝：

            [ERROR] Cannot push a new trajectory while another is being executed
            [ERROR] Apparently trajectory initialization failed

        对外表现为 MoveIt 错误码 -4 (CONTROL_FAILED)，并且会**污染后续一长串尝试**
        ——实测一次下降超时导致之后 11 次尝试全部 -4，直到旧轨迹自己跑完为止。
        这里先 cancel_goal_async，再等 result future 落地（= 控制器已停止），
        使单次超时与后续尝试互相隔离。
        """
        cancel_future = goal_handle.cancel_goal_async()
        self._wait(cancel_future, 5.0)
        self._wait(result_future, drain_timeout)

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
        # 显式声明"起态与当前状态无差异"。
        # 默认构造的 RobotState(is_diff=False) 会被 move_group 判为"显式提供了起态"，
        # 抛出告警：
        #   Execution of motions should always start at the robot's current state.
        #   Ignoring the state supplied as start state in the motion planning request
        # is_diff=True 表示"相对当前状态的差异为空"，语义与 MoveIt 实际采用的
        # 当前状态完全一致，因此不改变任何运动行为。
        request.start_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        # 同理：默认的 planning_scene_diff.robot_state 也是 is_diff=False，
        # 会触发配套告警 "...as difference in the planning scene diff"。
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self._client.send_goal_async(goal)
        if not self._wait(send_future, 10.0):
            return False, '发送运动目标超时'
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, 'move_group 拒绝该运动目标'

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, timeout):
            self._abort_and_drain(goal_handle, result_future)
            return False, f'运动执行超时（>{timeout:.0f}s，已取消并等待控制器停下）'

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
        ok, reason = self._execute(request, timeout)
        self._record_plan(ok)
        return ok, reason

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
        ok, reason = self._execute(request, timeout)
        self._record_plan(ok)
        return ok, reason

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

    def compute_cartesian_fraction(self, waypoints, avoid_collisions: bool = True,
                                   max_step: float = 0.003, timeout: float = 15.0):
        """仅计算笛卡尔路径并返回完成比例（**不执行**），用于定位下压被谁拦截。

        以当前状态为起点；先按 avoid_collisions=True 与 False 各算一次：
            忽略碰撞后接近 100%、含碰撞偏低 => 被碰撞体拦截；
            两者都偏低                        => 关节限位 / 逆解不可达。
        返回 (fraction, 说明)；fraction 为 None 表示服务不可用或超时。
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            return None, 'compute_cartesian_path 服务不可用'

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = list(waypoints)
        request.max_step = float(max_step)
        request.jump_threshold = 0.0
        request.avoid_collisions = bool(avoid_collisions)
        request.start_state.is_diff = True

        future = self._cartesian_client.call_async(request)
        if not self._wait(future, timeout):
            return None, '笛卡尔路径计算超时'
        response = future.result()
        if response is None:
            return None, '笛卡尔路径计算无响应'
        return float(response.fraction), 'ok'

    def compute_fk(self, joint_names, positions, link_name: str, timeout: float = 5.0):
        """用 MoveIt 的正运动学求指定连杆在世界系的位姿，返回 (x,y,z,qx,qy,qz,qw) 或 None。"""
        if not self._fk_client.wait_for_service(timeout_sec=5.0):
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = self.base_frame
        request.fk_link_names = [link_name]
        request.robot_state.joint_state.name = list(joint_names)
        request.robot_state.joint_state.position = [float(p) for p in positions]
        future = self._fk_client.call_async(request)
        if not self._wait(future, timeout):
            return None
        response = future.result()
        if response is None or not response.pose_stamped:
            return None
        pose = response.pose_stamped[0].pose
        return (pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)

    def cartesian_collision_report(self, waypoints, joint_names, max_step: float = 0.003,
                                   timeout: float = 15.0):
        """定位下压被"谁"拦截。

        先算一条**忽略碰撞**的完整笛卡尔路径（可到终点），再沿该路径自起点逐个
        关节状态调用 /check_state_validity，返回**首次出现接触**的状态序号、接触对
        与此时末端相对起点的完成比例。这样能直接指出挡住下压的碰撞体。
        返回 dict（at/contacts/ee_pose/finger_pose）或 None。
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = list(waypoints)
        request.max_step = float(max_step)
        request.jump_threshold = 0.0
        request.avoid_collisions = False   # 走完整条路径，便于定位首个碰撞点
        request.start_state.is_diff = True

        future = self._cartesian_client.call_async(request)
        if not self._wait(future, timeout):
            return None
        response = future.result()
        if response is None:
            return None

        points = response.solution.joint_trajectory.points
        if not points:
            return None
        if not self._validity_client.wait_for_service(timeout_sec=5.0):
            return None

        total = len(points)
        for index, point in enumerate(points):
            state = RobotState()
            state.joint_state.name = list(joint_names)
            state.joint_state.position = [float(p) for p in point.positions]
            validity = GetStateValidity.Request()
            validity.robot_state = state
            validity.group_name = self.group_name

            vfuture = self._validity_client.call_async(validity)
            if not self._wait(vfuture, 5.0):
                continue
            vresponse = vfuture.result()
            if vresponse is None or vresponse.valid:
                continue
            contacts = []
            for c in vresponse.contacts:
                contacts.append(
                    f'{c.contact_body_1 or "?"}<->{c.contact_body_2 or "?"}'
                    f'@({c.position.x * 1000:.0f},{c.position.y * 1000:.0f},'
                    f'{c.position.z * 1000:.0f})mm depth={c.depth * 1000:.1f}mm'
                )
            ee_pose = self.compute_fk(joint_names, point.positions, self.ee_link)
            finger_pose = self.compute_fk(joint_names, point.positions, 'panda_leftfinger')
            return {
                'at': float(index) / float(max(total - 1, 1)),
                'contacts': contacts,
                'ee_pose': ee_pose,
                'finger_pose': finger_pose,
            }
        return None

    def compute_ik(self, pose: Pose, avoid_collisions: bool = False,
                   seed_positions=None, timeout: float = 5.0):
        """调用 ``/compute_ik`` 求解给定位姿的逆解（诊断用，不影响任何运动）。

        ``avoid_collisions=False`` 只问"这个位姿在运动学上解不解得出来"；
        ``=True`` 额外要求解必须是**无碰撞**的——两者对比即可把失败切成两类：
            仅 True 失败 => 有逆解但落在碰撞里（目标位姿本身撞东西）；
            两者都失败   => 关节限位/姿态约束下根本没有逆解。
        ``seed_positions`` 为 None 时以"当前状态"为种子（is_diff=True），
        否则用给定关节角作为种子。

        返回 dict：{'error': int, 'error_name': str, 'joint_names': [...],
                    'positions': [...], 'contacts': [...]}；服务不可用时返回 None。
        """
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            return None

        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.group_name
        ik.ik_link_name = self.ee_link
        ik.pose_stamped.header.frame_id = self.base_frame
        ik.pose_stamped.pose = pose
        ik.avoid_collisions = bool(avoid_collisions)
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        if seed_positions is None:
            ik.robot_state.is_diff = True
        else:
            ik.robot_state.joint_state.name = list(self.joint_names)
            ik.robot_state.joint_state.position = [float(p) for p in seed_positions]
        request.ik_request = ik

        future = self._ik_client.call_async(request)
        if not self._wait(future, timeout + 5.0):
            return None
        response = future.result()
        if response is None:
            return None
        code = response.error_code.val
        # 注意：GetPositionIK 的响应只有 solution(RobotState) + error_code，
        # RobotState 本身**没有** contacts 字段（接触信息只有 /check_state_validity 有）。
        return {
            'error': code,
            'error_name': moveit_error_name(code),
            'joint_names': list(response.solution.joint_state.name),
            'positions': list(response.solution.joint_state.position),
            'contacts': [],
        }

    def check_state_validity(self, joint_names, positions=None, timeout: float = 5.0):
        """调用 ``/check_state_validity`` 判断某关节状态是否无碰撞。

        ``positions`` 为 None 时检查"当前状态"——这是 OMPL 的起点是否合法的直接依据
        （起点自碰撞会让 RRTConnect 直接判定无法采样，表现为通用错误码 99999）。
        返回 dict：{'valid': bool, 'contacts': [...]}；服务不可用时返回 None。
        """
        if not self._validity_client.wait_for_service(timeout_sec=5.0):
            return None

        request = GetStateValidity.Request()
        if positions is None:
            request.robot_state.is_diff = True
        else:
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [float(p) for p in positions]
        request.group_name = self.group_name

        future = self._validity_client.call_async(request)
        if not self._wait(future, timeout):
            return None
        response = future.result()
        if response is None:
            return None
        contacts = []
        for c in response.contacts:
            contacts.append(
                f'{c.contact_body_1 or "?"}<->{c.contact_body_2 or "?"}'
                f'@({c.position.x * 1000:.0f},{c.position.y * 1000:.0f},'
                f'{c.position.z * 1000:.0f})mm depth={c.depth * 1000:.1f}mm'
            )
        return {'valid': bool(response.valid), 'contacts': contacts}

    def move_cartesian(self, waypoints, velocity: float = 0.15, max_step: float = 0.004,
                       min_fraction: float = 0.9, timeout: float = 40.0,
                       avoid_collisions: bool = True):
        """沿直线插补运动到给定路点（用于下降抓取、垂直抬升与放置）。

        各阶段（含下压抓取）统一用 ``avoid_collisions=True``："夹爪 <-> 当前目标物体"
        的必然接触由动态 ACM 在规划前显式豁免（见节点 _acm_set_target），因此下压
        不再需要整体关闭碰撞检测，仍会防撞桌、防扫到邻物。

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
        request.avoid_collisions = bool(avoid_collisions)
        # 显式声明"起态与当前状态无差异"。
        # 默认构造的空 RobotState(is_diff=False) 会被 cartesian 服务尝试做消息转换，
        # 从而报：[ERROR] [moveit_robot_state.conversions] Found empty JointState message
        # is_diff=True 表示以当前状态为起点，与 MoveIt 的实际回退行为一致。
        request.start_state.is_diff = True

        future = self._cartesian_client.call_async(request)
        if not self._wait(future, 15.0):
            return False, '笛卡尔路径计算超时'
        response = future.result()
        if response is None:
            return False, '笛卡尔路径计算无响应'
        if response.fraction < min_fraction:
            self._record_plan(False)
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
            self._abort_and_drain(goal_handle, result_future)
            return False, f'轨迹执行超时（>{timeout:.0f}s，已取消并等待控制器停下）'
        wrapped = result_future.result()
        if wrapped is None:
            return False, '未收到轨迹执行结果'
        code = wrapped.result.error_code.val
        if code == MOVEIT_SUCCESS:
            self._record_plan(True)
            return True, f'ok (fraction={response.fraction:.2f})'
        self._record_plan(False)
        return False, f'轨迹执行 MoveIt 错误码 {code}'

    # ------------------------------------------------------------------
    # Allowed Collision Matrix（ACM）
    # ------------------------------------------------------------------
    def get_scene_objects(self, timeout: float = 5.0):
        """直接向 move_group 查询"世界系碰撞体 id/位姿"与"附着在机器人上的物体 id"。

        用途：诊断**规划场景里残留的悬空碰撞盒**。这类盒子在节点自己的账本
        （_scene_object_ids/_attached_ids）里查不到，却会卡死后续规划，必须直接问
        move_group。返回 ``(world, attached)``：world 为 [(id, x, y, z), ...]，
        attached 为 [id, ...]；服务不可用返回 None。
        """
        if not self._scene_get_client.wait_for_service(timeout_sec=timeout):
            return None
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        future = self._scene_get_client.call_async(request)
        if not self._wait(future, timeout):
            return None
        response = future.result()
        if response is None:
            return None
        world = []
        for obj in response.scene.world.collision_objects:
            pose = obj.primitive_poses[0] if obj.primitive_poses else None
            if pose is None:
                world.append((obj.id, float('nan'), float('nan'), float('nan')))
            else:
                world.append((obj.id, pose.position.x, pose.position.y, pose.position.z))
        attached = [a.object.id for a in response.scene.robot_state.attached_collision_objects]
        return world, attached

    def get_allowed_collision_matrix(self, timeout: float = 5.0):
        """读取 move_group 当前的**完整** ACM，返回 ``(entry_names, entry_values)``。

        entry_values 为与 entry_names 同序的方阵（``list[list[bool]]``，True=允许碰撞）；
        服务不可用或超时返回 None。

        为什么要读全量：对 ACM 必须做**增量**修改（只改夹爪-目标物体那几项）后整体写回。
        若只发送少数条目，会被 move_group 当作整体替换，抹掉机器人原有的自碰撞豁免
        （相邻连杆、指垫等），使机械臂被判为自碰撞而完全无法运动——本项目已踩过该坑。
        """
        if not self._scene_get_client.wait_for_service(timeout_sec=timeout):
            return None
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._scene_get_client.call_async(request)
        if not self._wait(future, timeout):
            return None
        response = future.result()
        if response is None:
            return None
        matrix = response.scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        values = [list(entry.enabled) for entry in matrix.entry_values]
        return names, values