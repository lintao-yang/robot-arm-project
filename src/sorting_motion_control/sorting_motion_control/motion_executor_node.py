"""模块5 运动规划与控制节点 + 模块6 闭环反馈（抓取前校正 / 抓取确认 / 放置复核）。

对外提供 /sorting/execute_pick_place 服务，内部完成一次完整的
"预抓取 -> （可选二次视觉校正）-> 抓取 -> 抓取确认 -> 抬升 -> 放置 -> 释放 -> 复核" 动作链。

闭环体现在三处：
  1. 抓取前二次视觉校正：到达预抓取点后重新检测目标并重规划抓取位姿
  2. 抓取确认：闭合后由手指关节位置判定是否夹住；抬升后再判一次以发现滑落
  3. 放置复核：释放并退回后重新检测，确认目标已进入对应分拣区

搬运保持：Gazebo Sim 下机械臂由 gz_ros2_control 以真实力矩驱动（每关节 PID），
接触力能反传到关节，靠指尖摩擦 + effort 力控夹爪即可夹持搬运；
抓取确认、抬升后滑落检测等闭环判据均为真实物理量。
"""

import copy
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import DetectionArray, GraspCandidate
from sorting_interfaces.srv import ExecutePickPlace, PlanGrasp

from .gripper import GripperController
from .moveit_client import MoveGroupClient


class MotionExecutorNode(Node):
    """抓取-放置动作执行与闭环反馈。"""

    def __init__(self):
        super().__init__('motion_executor_node')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        frames = self.system_config['frames']
        motion_cfg = self.system_config['motion']
        gripper_cfg = self.system_config['gripper']
        topics = self.system_config['topics']
        scene_cfg = self.system_config['scene']

        self.group = ReentrantCallbackGroup()

        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7',
        ]
        self.moveit = MoveGroupClient(
            self,
            group_name='panda_arm',
            ee_link=frames['ee_link'],
            base_frame=frames['base'],
            joint_names=self.joint_names,
            planner_id=motion_cfg.get('planner_id', 'RRTConnect'),
            position_tolerance=float(motion_cfg.get('goal_position_tolerance', 0.006)),
            orientation_tolerance=float(motion_cfg.get('goal_orientation_tolerance', 0.08)),
            planning_time=float(motion_cfg.get('planning_time', 5.0)),
            planning_attempts=int(motion_cfg.get('planning_attempts', 5)),
        )
        self.gripper = GripperController(self, gripper_cfg)

        self.velocity = float(motion_cfg.get('velocity_scaling', 0.25))
        self.acceleration = float(motion_cfg.get('accel_scaling', 0.25))
        self.approach_velocity = float(motion_cfg.get('approach_velocity_scaling', 0.10))
        self.cartesian_velocity = float(motion_cfg.get('cartesian_velocity_scaling', 0.15))
        self.home_joint_values = list(motion_cfg.get('home_joint_values', []))
        self.refine_tolerance = float(self.system_config['task'].get('refine_tolerance', 0.015))

        self.table_top_z = float(scene_cfg['table']['top_z'])
        self.table_size = list(scene_cfg['table']['size'])
        self.table_pose = list(scene_cfg['table']['pose'])
        self.bin_base_thickness = float(self.system_config['grasp'].get('bin_base_thickness', 0.008))

        # ---- 最新一帧带三维定位的检测结果（用于闭环校正与放置复核）----
        self._latest_targets = None
        self._targets_seq = 0
        self.create_subscription(
            DetectionArray, topics['targets'], self._on_targets, 5,
            callback_group=self.group,
        )

        # ---- 抓取规划服务客户端（二次校正时复用抓取决策逻辑）----
        self.plan_client = self.create_client(
            PlanGrasp, self.system_config['services']['plan_grasp'],
            callback_group=self.group,
        )

        # ---- 规划场景服务客户端（加入桌面与分拣箱碰撞体）----
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self.group
        )

        self.create_service(
            ExecutePickPlace,
            self.system_config['services']['execute_pick_place'],
            self.on_execute_pick_place,
            callback_group=self.group,
        )

        # 启动后延时执行：加载碰撞环境 + 回到初始位姿
        self._bootstrapped = False
        self._bootstrapping = False
        self.create_timer(2.0, self.bootstrap, callback_group=self.group)

        self.get_logger().info('运动执行节点启动')

    # ==================================================================
    # 启动引导
    # ==================================================================
    def bootstrap(self):
        # 防止定时器重入占用 executor 线程（存储过程最长可达数十秒）
        if self._bootstrapped or self._bootstrapping:
            return
        self._bootstrapping = True
        try:
            if not self.moveit.wait_for_server(timeout=5.0):
                self.get_logger().warn('等待 /move_action 服务中...')
                return

            self._setup_planning_scene()
            if self.home_joint_values:
                ok, message = self.moveit.move_to_joint_values(
                    self.home_joint_values, self.velocity, self.acceleration, timeout=30.0
                )
                self.get_logger().info(f'回到初始位姿: {ok} ({message})')
            self.gripper.open()
            self._bootstrapped = True
            self.get_logger().info('运动执行节点就绪')
        finally:
            self._bootstrapping = False

    def _setup_planning_scene(self):
        """把桌面与分拣箱作为碰撞体加入规划场景。"""
        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = self.system_config['frames']['world']
        table.id = 'work_table'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(v) for v in self.table_size]
        table.primitives.append(box)
        table_pose = Pose()
        table_pose.position.x = float(self.table_pose[0])
        table_pose.position.y = float(self.table_pose[1])
        table_pose.position.z = float(self.table_pose[2])
        table_pose.orientation.w = 1.0
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table)

        bin_size = 0.16
        bin_height = self.bin_base_thickness + 0.03
        for zone_name in self.system_config['scene'].get('bins', []):
            zone_pose = self.class_table.zone_pose(zone_name)
            obstacle = CollisionObject()
            obstacle.header.frame_id = self.system_config['frames']['world']
            obstacle.id = f'bin_{zone_name}'
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [bin_size, bin_size, bin_height]
            obstacle.primitives.append(primitive)
            pose = Pose()
            pose.position.x = float(zone_pose[0])
            pose.position.y = float(zone_pose[1])
            pose.position.z = bin_height / 2.0
            pose.orientation.w = 1.0
            obstacle.primitive_poses.append(pose)
            obstacle.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obstacle)

        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/apply_planning_scene 不可用，跳过碰撞环境配置')
            return
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if future.done() and future.result() is not None:
            self.get_logger().info(f'规划场景已加入桌面与 {len(scene.world.collision_objects) - 1} 个分拣箱')
        else:
            self.get_logger().warn('规划场景配置失败')

    # ==================================================================
    # 检测结果缓存
    # ==================================================================
    def _on_targets(self, msg: DetectionArray):
        self._latest_targets = msg
        self._targets_seq += 1

    def _wait_fresh_targets(self, timeout: float = 3.0):
        """等待一帧比当前更新的检测结果。"""
        seq0 = self._targets_seq
        deadline = time.time() + timeout
        while time.time() < deadline and self._targets_seq <= seq0 and rclpy.ok():
            time.sleep(0.05)
        return self._latest_targets if self._targets_seq > seq0 else None

    # ==================================================================
    # 闭环步骤 1：抓取前二次视觉校正
    # ==================================================================
    def _refine_grasp(self, candidate: GraspCandidate, expected_xy):
        msg = self._wait_fresh_targets(timeout=3.0)
        if msg is None:
            return None, '未获取到新的检测结果'

        best = None
        best_distance = 1e9
        for detection in msg.detections:
            if not detection.has_3d or int(detection.class_id) != int(candidate.class_id):
                continue
            dx = detection.position_base.x - expected_xy[0]
            dy = detection.position_base.y - expected_xy[1]
            distance = (dx * dx + dy * dy) ** 0.5
            if distance < best_distance:
                best_distance = distance
                best = detection

        if best is None:
            return None, '二次识别未找到同类目标'
        if best_distance > 0.10:
            return None, f'二次识别目标位置偏差过大（{best_distance * 1000:.0f}mm）'

        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            return None, '抓取规划服务不可用'
        request = PlanGrasp.Request(target=best)
        future = self.plan_client.call_async(request)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done() or future.result() is None or not future.result().success:
            return None, '二次校正重规划失败'

        refined = future.result().candidate
        offset = (
            (refined.grasp_pose.position.x - candidate.grasp_pose.position.x) ** 2
            + (refined.grasp_pose.position.y - candidate.grasp_pose.position.y) ** 2
        ) ** 0.5
        return refined, f'校正量 {offset * 1000:.1f}mm'

    # ==================================================================
    # 闭环步骤 3：放置后视觉复核
    # ==================================================================
    def _verify_placement(self, class_id: int, place_xy, tolerance: float = 0.09) -> bool:
        msg = self._wait_fresh_targets(timeout=3.0)
        if msg is None:
            return False
        for detection in msg.detections:
            if not detection.has_3d or int(detection.class_id) != int(class_id):
                continue
            dx = detection.position_base.x - place_xy[0]
            dy = detection.position_base.y - place_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < tolerance:
                return True
        return False

    # ==================================================================
    # 主服务：执行一次抓取-放置
    # ==================================================================
    def on_execute_pick_place(self, request, response):
        start_time = time.time()
        candidate = request.candidate
        use_closed_loop = bool(request.use_closed_loop)

        retries = 0
        grasped = False
        place_verified = False
        message = ''

        if not self.moveit.wait_for_server(timeout=30.0):
            response.success = False
            response.message = '/move_action 服务不可用'
            response.duration = 0.0
            return response

        grasp_pose = copy.deepcopy(candidate.grasp_pose)
        pregrasp_pose = copy.deepcopy(candidate.pregrasp_pose)
        lift_pose = copy.deepcopy(candidate.lift_pose)
        place_pose = copy.deepcopy(candidate.place_pose)

        expected_xy = (grasp_pose.position.x, grasp_pose.position.y)

        # ---- 0) 张开夹爪 ----
        self.gripper.open()

        # ---- 1) 移动到预抓取点 ----
        ok, reason = self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
        if not ok:
            response.success = False
            response.message = f'预抓取点规划失败: {reason}'
            response.duration = time.time() - start_time
            return response

        # ---- 2) 闭环：抓取前二次视觉校正 ----
        if use_closed_loop:
            refined, refine_message = self._refine_grasp(candidate, expected_xy)
            self.get_logger().info(f'二次视觉校正: {refine_message}')
            if refined is not None:
                grasp_pose = copy.deepcopy(refined.grasp_pose)
                lift_pose = copy.deepcopy(refined.lift_pose)
                message += '已执行抓取前二次校正; '
            else:
                message += f'二次校正跳过({refine_message}); '

        # ---- 3) 直线下降至抓取位姿（低速，避免撞飞目标）----
        ok, reason = self.moveit.move_to_pose(
            grasp_pose, self.approach_velocity, self.approach_velocity,
            position_tolerance=0.004, orientation_tolerance=0.05,
        )
        if not ok:
            # 位姿目标规划失败时退化为直线插补
            ok, reason = self.moveit.move_cartesian(
                [grasp_pose], velocity=self.cartesian_velocity, max_step=0.003, timeout=30.0
            )
        if not ok:
            self.gripper.open()
            self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
            response.success = False
            response.message = f'抓取位姿不可达: {reason}'
            response.duration = time.time() - start_time
            return response

        # ---- 4) 闭合夹爪 + 抓取确认（手指位置判据）----
        finger_position = self.gripper.close_and_measure()
        grasped = self.gripper.is_grasped(finger_position)
        self.get_logger().info(
            f'夹爪闭合: 手指位置={finger_position if finger_position is None else round(finger_position, 4)} '
            f'阈值={self.gripper.grasp_threshold} => {"夹住" if grasped else "空夹"}'
        )

        if not grasped:
            self.gripper.open()
            self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
            response.success = True
            response.grasped = False
            response.place_verified = False
            response.retries = retries
            response.duration = time.time() - start_time
            response.message = message + '抓取失败：夹爪闭合到底，未夹住物体'
            return response

        # 抬升与搬运全程保持夹持力矩（力控指令需持续有效，否则物体会在加速阶段滑落）
        self.gripper.hold_closing()

        # ---- 5) 垂直直线抬升 + 滑落检测（必须走直线，否则路径绕行会把物体拖掉）----
        ok, reason = self.moveit.move_cartesian(
            [lift_pose], velocity=self.cartesian_velocity, max_step=0.003, timeout=40.0
        )
        if not ok:
            ok, reason = self.moveit.move_to_pose(
                lift_pose, self.approach_velocity, self.approach_velocity
            )
        if not ok:
            self.gripper.open()
            response.success = False
            response.message = message + f'抬升失败: {reason}'
            response.duration = time.time() - start_time
            return response

        time.sleep(0.3)
        after_lift = self.gripper.finger_position()
        held = self.gripper.is_grasped(after_lift)
        self.get_logger().info(
            f'抬升后手指位置 = {after_lift if after_lift is None else round(after_lift, 4)} '
            f'(闭合时 {finger_position if finger_position is None else round(finger_position, 4)}) '
            f'=> {"保持" if held else "滑落"}'
        )
        if not held:
            self.gripper.open()
            self.moveit.move_to_pose(pregrasp_pose, self.velocity, self.acceleration)
            response.success = True
            response.grasped = False
            response.retries = retries
            response.duration = time.time() - start_time
            response.message = (
                message + f'抓取失败：抬升后检测到物体滑落（手指位置 '
                f'{after_lift if after_lift is None else round(after_lift, 4)}）'
            )
            return response

        # ---- 6) 移动到放置点上方 ----
        pre_place = copy.deepcopy(place_pose)
        pre_place.position.z += 0.10
        ok, reason = self.moveit.move_to_pose(pre_place, self.velocity, self.acceleration)
        if not ok:
            self.gripper.open()
            response.success = False
            response.message = message + f'放置点上方规划失败: {reason}'
            response.duration = time.time() - start_time
            return response

        # ---- 7) 直线下降到放置点并释放 ----
        ok, reason = self.moveit.move_cartesian(
            [place_pose], velocity=self.cartesian_velocity, max_step=0.003, timeout=30.0
        )
        if not ok:
            ok, reason = self.moveit.move_to_pose(
                place_pose, self.approach_velocity, self.approach_velocity,
                position_tolerance=0.008, orientation_tolerance=0.10,
            )
        if not ok:
            self.get_logger().warn(f'放置位姿不可达({reason})，改为在当前位置释放')

        self.gripper.release()

        # ---- 8) 直线退回上方 ----
        ok, reason = self.moveit.move_cartesian(
            [pre_place], velocity=self.cartesian_velocity, max_step=0.004, timeout=30.0
        )
        if not ok:
            self.moveit.move_to_pose(pre_place, self.velocity, self.acceleration)

        # ---- 9) 闭环：放置后视觉复核 ----
        place_verified = self._verify_placement(
            int(candidate.class_id), (place_pose.position.x, place_pose.position.y)
        )

        response.success = True
        response.grasped = True
        response.place_verified = bool(place_verified)
        response.retries = retries
        response.duration = time.time() - start_time
        response.message = message + (
            '分拣完成' if place_verified else '已放置但视觉复核未确认目标在分拣区内'
        )
        self.get_logger().info(
            f'{candidate.class_name} -> {candidate.placement_zone} '
            f'成功={response.success} 抓取={grasped} 复核={place_verified} '
            f'耗时={response.duration:.2f}s'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutorNode()
    # 线程数需大于"阻塞式服务回调数"，否则响应回调查询无可用线程
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()