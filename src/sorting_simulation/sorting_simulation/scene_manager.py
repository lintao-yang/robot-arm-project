"""模块1 仿真环境管理节点。

职责：
  1. 生成 / 重置分拣场景（分拣箱 + 多类待分拣物体），支持域随机化
  2. 通过 Gazebo Sim 的 set_pose 服务（gz CLI）施加光照与相机安装位姿的随机扰动
  3. 发布真值 /sorting/ground_truth（供合成数据标注与分拣效果评估使用）
  4. 广播坐标变换：world -> panda_link0、world -> camera_optical_frame
     （相机光学外参由安装位姿 + 标准光学旋转推导，等价于仿真中的"外参标定"）

注意：真值仅用于"离线标注"与"结果评估"，在线抓取链路完全不使用真值，
      以保证视觉闭环的含金量（见 teach.txt 的反面案例：真值瞬移抓取）。
"""

import math
import os
import subprocess
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from sorting_bringup import ClassTable, load_system_config
from sorting_interfaces.msg import ObjectState, ObjectStateArray
from sorting_interfaces.srv import ResetScene
from tf2_msgs.msg import TFMessage

# 相机 body 系（x 前 / y 左 / z 上）-> 光学系（z 前 / x 右 / y 下）的标准旋转
R_BODY_TO_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


def _inertia_box(mass: float, size: List[float]) -> Dict[str, float]:
    a, b, c = size
    return {
        'ixx': mass * (b * b + c * c) / 12.0,
        'iyy': mass * (a * a + c * c) / 12.0,
        'izz': mass * (a * a + b * b) / 12.0,
    }


def _inertia_cylinder(mass: float, diameter: float, height: float) -> Dict[str, float]:
    radius = diameter / 2.0
    ixx = mass * (3.0 * radius * radius + height * height) / 12.0
    return {'ixx': ixx, 'iyy': ixx, 'izz': mass * radius * radius / 2.0}


def _inertia_sphere(mass: float, diameter: float) -> Dict[str, float]:
    inertia = 2.0 * mass * (diameter / 2.0) ** 2 / 5.0
    return {'ixx': inertia, 'iyy': inertia, 'izz': inertia}


class SceneManager(Node):
    """Gazebo 场景与真值管理。"""

    def __init__(self):
        super().__init__('scene_manager')

        self.system_config = load_system_config()
        self.class_table = ClassTable()
        self.scene_cfg = self.system_config['scene']
        self.dr_cfg = self.class_table.domain_randomization or {}

        # use_sim_time 由 rclpy 自动声明，这里不再重复声明
        self.declare_parameter('object_mass', 0.06)

        self.object_mass = float(self.get_parameter('object_mass').value)
        self.rng = np.random.default_rng(int(time.time()))

        self._group = ReentrantCallbackGroup()

        # ---- Gazebo Sim 实体操作 ----
        # 说明（重要）：Gazebo Sim 提供的 /world/<world>/create|remove|set_pose 服务
        # 在本环境下**无法从 rclpy 客户端调用**——服务能被图查询看到，但
        # rclpy 的 service_is_ready() 恒为 False、call_async 永不返回
        # （C++ 的 ros_gz_sim 工具调用同一服务则正常，属 rclpy 侧的服务匹配问题）。
        # 因此这里不走 ROS 服务，改用两条已验证可用的进程外通道：
        #   生成：ros2 run ros_gz_sim create -file <临时SDF> ...
        #   删除/改位姿：ign service -s /world/<world>/{remove,set_pose}
        self.world_name = str(
            self.system_config['scene'].get('gz_world_name', 'sorting_world')
        )

        # ---- 重置场景服务 ----
        self.create_service(
            ResetScene,
            self.system_config['services']['reset_scene'],
            self.on_reset_scene,
            callback_group=self._group,
        )

        # ---- 真值发布 ----
        self.pub_ground_truth = self.create_publisher(
            ObjectStateArray, self.system_config['topics']['ground_truth'], 10
        )

        # 物体实时位姿：Gazebo Sim 通过 /world/<world>/dynamic_pose/info 发布
        # （gz 的 Pose_V），由 ros_gz_bridge 转成 tf2_msgs/TFMessage，
        # child_frame_id 形如 "<模型名>" 或 "<模型名>/<连杆名>"。
        # 该话题按物理频率（数百 Hz）发布，必须单独放在一个**互斥**回调组里：
        # 否则这些高频回调会把 MultiThreadedExecutor 的线程全部占住，
        # 定时器（真值发布 / 相机 TF）永远排不上队（实测真值话题一条不发）。
        self._pose_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            TFMessage, f'/world/{self.world_name}/dynamic_pose/info',
            self.on_pose_tf, 1, callback_group=self._pose_group,
        )

        # ---- TF ----
        # 说明：world -> panda_link0 由 robot_state_publisher 依据 URDF 中的
        # panda_base_mount 固定关节发布；此处只负责相机光学外参。
        self.dynamic_tf = TransformBroadcaster(self)

        # ---- 状态 ----
        self._objects: Dict[str, Dict] = {}      # 模型名 -> 元信息
        self._model_poses: Dict[str, Pose] = {}
        self._bins_done = set()
        self._bins_spawned = False
        self._initial_objects_done = False
        self._max_setup_attempts = 3
        self._known_models = set()
        self._camera_pose = self._base_camera_pose()
        self._setup_lock = threading.Lock()

        self.create_timer(0.1, self.publish_state, callback_group=self._group)
        # 场景装配（生成分拣箱与物体）必须在**独立线程**中顺序执行：
        # 若放在定时器回调里阻塞等待 Gazebo 服务，同一回调组的定时器会反复进入
        # 并阻塞在同一把锁上，最终耗尽 MultiThreadedExecutor 的线程池，
        # 导致服务响应回调无线程可执行、请求永远超时。
        self._setup_thread = threading.Thread(target=self._setup_worker, daemon=True)
        self._setup_thread.start()

    # ==================================================================
    # 基础工具
    # ==================================================================
    def _base_camera_pose(self) -> List[float]:
        mount = self.system_config['camera']['mount_pose']
        return [mount['x'], mount['y'], mount['z'],
                mount['roll'], mount['pitch'], mount['yaw']]

    def _run(self, command: List[str], timeout: float = 15.0):
        """在子进程中执行 Gazebo Sim 的 CLI，返回 (是否成功, 合并输出)。"""
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=timeout
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().error(f'命令执行失败({command[:4]}): {exc}')
            return False, ''
        output = (result.stdout or '') + (result.stderr or '')
        return result.returncode == 0, output

    def _ign_service(self, service: str, request_text: str, request_type: str,
                     timeout_ms: int = 3000) -> bool:
        """调用 gz-transport 服务（绕开 ROS 服务层，原因见构造函数说明）。"""
        ok, output = self._run(
            ['ign', 'service', '-s', f'/world/{self.world_name}/{service}',
             '--reqtype', request_type, '--reptype', 'ignition.msgs.Boolean',
             '--timeout', str(timeout_ms), '--req', request_text],
            timeout=timeout_ms / 1000.0 + 6.0,
        )
        return ok and 'true' in output.lower()

    # ==================================================================
    # TF
    # ==================================================================
    def _publish_camera_tf(self):
        """world -> camera_optical_frame：外参来自相机安装位姿 + 光学系旋转。"""
        x, y, z, roll, pitch, yaw = self._camera_pose
        rotation_body = tf_transformations.euler_matrix(roll, pitch, yaw)[:3, :3]
        rotation_optical = rotation_body @ R_BODY_TO_OPTICAL

        quaternion = tf_transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_optical, np.zeros((3, 1))]), [0, 0, 0, 1]])
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.system_config['frames']['world']
        transform.child_frame_id = self.system_config['frames']['camera_optical']
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.dynamic_tf.sendTransform(transform)

    # ==================================================================
    # 模型生成
    # ==================================================================
    def _spawn_model(self, name: str, sdf: str, pose=None, timeout: float = 25.0) -> bool:
        """生成模型；pose 为 [x, y, z, roll, pitch, yaw]（world 坐标系）。

        走 `ros2 run ros_gz_sim create -file`（C++ 客户端，已验证可用）。
        """
        sdf_path = os.path.join('/tmp', f'sorting_spawn_{name}.sdf')
        try:
            with open(sdf_path, 'w', encoding='utf-8') as handle:
                handle.write(sdf)
        except OSError as exc:
            self.get_logger().error(f'写临时 SDF 失败: {exc}')
            return False

        command = ['ros2', 'run', 'ros_gz_sim', 'create',
                   '-file', sdf_path, '-name', name, '-world', self.world_name]
        if pose is not None:
            command += [
                '-x', repr(float(pose[0])), '-y', repr(float(pose[1])),
                '-z', repr(float(pose[2])), '-R', repr(float(pose[3])),
                '-P', repr(float(pose[4])), '-Y', repr(float(pose[5])),
            ]
        ok, output = self._run(command, timeout=timeout)
        if not ok or 'OK creation of entity' not in output:
            self.get_logger().warn(f'{name} 生成失败: {output.strip()[-160:]}')
            return False
        return True

    def _delete_model(self, name: str) -> bool:
        return self._ign_service(
            'remove', f'name: "{name}", type: 2', 'ignition.msgs.Entity'
        )

    def _bin_sdf(self, name: str, rgba: List[float], outer: float = 0.16,
                 wall: float = 0.008, height: float = 0.03) -> str:
        half = outer / 2.0
        inner_half = half - wall
        base_thickness = 0.008
        color = f'{rgba[0]} {rgba[1]} {rgba[2]} 1.0'

        def part(part_name, size, pose):
            sx, sy, sz = size
            px, py, pz = pose
            return f'''
      <collision name="{part_name}_collision">
        <pose>{px} {py} {pz} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="{part_name}_visual">
        <pose>{px} {py} {pz} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>'''

        parts = [
            part('bottom', (outer, outer, base_thickness), (0, 0, base_thickness / 2.0)),
            part('wall_py', (outer, wall, height), (0, half - wall / 2.0, base_thickness + height / 2.0)),
            part('wall_ny', (outer, wall, height), (0, -(half - wall / 2.0), base_thickness + height / 2.0)),
            part('wall_px', (wall, inner_half * 2.0, height), (half - wall / 2.0, 0, base_thickness + height / 2.0)),
            part('wall_nx', (wall, inner_half * 2.0, height), (-(half - wall / 2.0), 0, base_thickness + height / 2.0)),
        ]
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="bin">
{''.join(parts)}
    </link>
  </model>
</sdf>'''

    def _object_sdf(self, name: str, shape: str, size: List[float],
                    rgba: List[float], pose: List[float]) -> str:
        sx, sy, sz = size
        if shape == 'cylinder':
            geometry = f'<cylinder><radius>{sx / 2.0}</radius><length>{sz}</length></cylinder>'
            inertia = _inertia_cylinder(self.object_mass, sx, sz)
        elif shape == 'sphere':
            geometry = f'<sphere><radius>{sx / 2.0}</radius></sphere>'
            inertia = _inertia_sphere(self.object_mass, sx)
        else:
            geometry = f'<box><size>{sx} {sy} {sz}</size></box>'
            inertia = _inertia_box(self.object_mass, size)

        color = f'{rgba[0]} {rgba[1]} {rgba[2]} 1.0'
        # 位姿通过 SpawnEntity.initial_pose 指定，这里不再写 <pose>
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{self.object_mass}</mass>
        <inertia>
          <ixx>{inertia['ixx']}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{inertia['iyy']}</iyy><iyz>0</iyz>
          <izz>{inertia['izz']}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geometry}</geometry>
        <surface>
          <friction><ode><mu>5.0</mu><mu2>5.0</mu2></ode></friction>
          <!-- 注意：不要设置 <max_vel>/<min_depth>（contact_max_correcting_vel）。
               该值过小会让接触无法及时推开手指，指尖会"穿过"物体导致夹不住。 -->
          <contact>
            <ode>
              <kp>1.0e6</kp><kd>100.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>{geometry}</geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>'''

    # ==================================================================
    # 域随机化
    # ==================================================================
    def _randomized_rgba(self, class_id: int) -> List[float]:
        base = np.array(self.class_table.rgba_of(class_id), dtype=float)[:3]
        if not self.dr_cfg.get('enabled', True):
            return list(base)
        value_range = self.dr_cfg.get('value_range', [0.55, 1.0])
        jitter = float(self.dr_cfg.get('hue_jitter', 0.0))
        # 用 RGB 通道扰动近似色相抖动，保持物体主色不变以便颜色分割后端可用
        scale = float(self.rng.uniform(value_range[0], value_range[1]))
        perturbed = base * scale
        noise = self.rng.uniform(-jitter * 2.0, jitter * 2.0, size=3)
        perturbed = np.clip(perturbed + noise * 0.5, 0.05, 1.0)
        return list(perturbed.astype(float))

    def _randomize_camera(self):
        x, y, z, roll, pitch, yaw = self._base_camera_pose()
        if not self.dr_cfg.get('enabled', True):
            self._camera_pose = [x, y, z, roll, pitch, yaw]
        else:
            j_xyz = self.dr_cfg.get('camera_jitter_xyz', [0.0, 0.0, 0.0])
            j_rpy = self.dr_cfg.get('camera_jitter_rpy', [0.0, 0.0, 0.0])
            self._camera_pose = [
                x + float(self.rng.uniform(-j_xyz[0], j_xyz[0])),
                y + float(self.rng.uniform(-j_xyz[1], j_xyz[1])),
                z + float(self.rng.uniform(-j_xyz[2], j_xyz[2])),
                roll + float(self.rng.uniform(-j_rpy[0], j_rpy[0])),
                pitch + float(self.rng.uniform(-j_rpy[1], j_rpy[1])),
                yaw + float(self.rng.uniform(-j_rpy[2], j_rpy[2])),
            ]
        self._apply_entity_pose('overhead_camera', self._camera_pose)

    def _randomize_light(self):
        if not self.dr_cfg.get('enabled', True):
            return
        yaw = float(self.rng.uniform(*self.dr_cfg.get('light_yaw_range', [0.0, 6.2832])))
        pitch = float(self.rng.uniform(*self.dr_cfg.get('light_pitch_range', [0.6, 1.35])))
        radius = float(self.dr_cfg.get('light_pose_radius', 3.0))
        self._apply_entity_pose('main_light', [0.0, 0.0, radius, 0.0, pitch, yaw])

    def _apply_entity_pose(self, name: str, pose: List[float]):
        """移动已存在的模型（用于光照 / 相机安装位的域随机化）。"""
        quaternion = tf_transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
        request_text = (
            f'name: "{name}", '
            f'position: {{x: {pose[0]}, y: {pose[1]}, z: {pose[2]}}}, '
            f'orientation: {{x: {quaternion[0]}, y: {quaternion[1]}, '
            f'z: {quaternion[2]}, w: {quaternion[3]}}}'
        )
        return self._ign_service('set_pose', request_text, 'ignition.msgs.Pose')

    # ==================================================================
    # 场景装配
    # ==================================================================
    def _wait_gz_ready(self, timeout: float) -> bool:
        """等待 gz-transport 侧的 /world/<world>/create 服务出现。"""
        target = f'/world/{self.world_name}/create'
        deadline = time.time() + timeout
        while time.time() < deadline:
            ok, output = self._run(['ign', 'service', '-l'], timeout=8.0)
            if ok and target in output:
                return True
            time.sleep(1.0)
        return False

    def _setup_worker(self):
        """独立线程：等待 Gazebo Sim 就绪 -> 生成静态分拣箱 -> 摆放初始物体。"""
        if not self._wait_gz_ready(120.0):
            self.get_logger().error('Gazebo Sim 服务长时间不可用，场景初始化中止')
            return
        self.get_logger().info('Gazebo 服务就绪，开始装配场景')

        with self._setup_lock:
            for attempt in range(1, self._max_setup_attempts + 1):
                self.ensure_bins()
                if not self._bins_spawned:
                    time.sleep(1.0)
                    continue
                if self._objects:
                    self._initial_objects_done = True
                    return
                count = int(self.scene_cfg.get('n_objects', 4))
                try:
                    # 初始摆放即满足"四种物体每种各一个"（不重复），槽位随机排列
                    spawned = self._spawn_objects(
                        count, randomize_class=False, randomize_layout=True
                    )
                except Exception as exc:
                    self.get_logger().error(f'初始物体生成异常: {exc}')
                    spawned = 0
                if spawned > 0:
                    self._initial_objects_done = True
                    self.get_logger().info(f'场景装配完成，共 {spawned} 个待分拣物体')
                    return
                self.get_logger().warn(
                    f'初始物体生成失败（第 {attempt}/{self._max_setup_attempts} 次），稍后重试'
                )
                time.sleep(2.0)
        self.get_logger().error(
            '场景装配失败；可手动调用 /sorting/reset_scene 重新摆放物体'
        )

    def ensure_bins(self):
        """分拣箱已由世界文件静态定义（Gazebo Sim 版），此处只做就绪标记。

        Classic 版是在运行时用 /spawn_entity 生成 4 个箱子；迁移到 Gazebo Sim 后
        把箱子放进 worlds/sorting_world_sim.sdf，减少运行时服务调用，
        也让仿真一开始就有完整的静态环境。
        """
        if self._bins_spawned:
            return
        bins = self.scene_cfg.get('bins', [])
        self._bins_done = {f'bin_{zone_name}' for zone_name in bins}
        self._bins_spawned = True
        self.get_logger().info(f'分拣箱由世界文件提供（{len(bins)} 个）')

    def _spawn_objects(self, count: int, randomize_class: bool,
                       randomize_layout: bool = False) -> int:
        """清空旧物体并按紧凑网格重新摆放待分拣物体。

        摆放策略（对应"紧凑且不穿模"需求）：
          * 物体排成 ncols 列网格；列宽 = 该列物体 x 尺寸最大值，
            行高 = 该行物体 y 尺寸最大值。
          * 同一列内物体对齐到列边界（第 0 列靠右、其余列靠左），
            同一行内物体对齐到行边界（首行靠下、其余行靠上），
            从而保证网格线两侧任意相邻物体的**边界间距严格等于 gap**
            （默认 1.5mm，落在 0.5~2mm 合理区间），既紧凑又不重叠。
          * 物体平放（roll=pitch=0、yaw=0），底面贴合桌面（仅留极小的
            rest_clearance 防止初始穿模），姿态稳定、无悬空、无倾斜。
        """
        for name in list(self._objects.keys()):
            self._delete_model(name)
        self._objects.clear()
        time.sleep(0.3)

        layout = self.scene_cfg.get('layout', {})
        class_ids = self.class_table.class_ids
        target = max(1, count)
        ncols = max(1, int(layout.get('columns', 2)))
        gap = float(layout.get('gap', 0.0015))
        center_x = float(layout.get('center_x', 0.40))
        center_y = float(layout.get('center_y', 0.0))
        clearance = float(layout.get('rest_clearance', 0.0005))

        picks: List[int] = []
        if randomize_class:
            # 随机类别（可重复）：种类最多样，但可能出现同色相邻
            picks = [int(self.rng.choice(class_ids)) for _ in range(target)]
        elif randomize_layout:
            # "每种各一个"但**槽位随机排列**：保证各类各一个、无同色相邻，同时让每帧的
            # "类别->槽位"映射不同（配合相机/光照随机化提供帧间多样性，供合成数据集采集）。
            # 网格几何与边界间隙保持不变。
            permutation = [int(cid) for cid in self.rng.permutation(class_ids)]
            while len(picks) < target:
                picks.extend(permutation)
            picks = picks[:target]
        else:
            picks = [int(class_ids[index % len(class_ids)]) for index in range(target)]
        sizes = [self.class_table.size_of(cid) for cid in picks]

        nrows = int(math.ceil(target / float(ncols)))
        col_w = [0.0] * ncols
        row_h = [0.0] * nrows
        for index, size in enumerate(sizes):
            col_w[index % ncols] = max(col_w[index % ncols], size[0])
            row_h[index // ncols] = max(row_h[index // ncols], size[1])

        total_w = sum(col_w) + gap * (ncols - 1)
        total_h = sum(row_h) + gap * (nrows - 1)
        left = center_x - total_w / 2.0
        top = center_y + total_h / 2.0

        col_left = []
        cursor = left
        for width in col_w:
            col_left.append(cursor)
            cursor += width + gap
        row_top = []
        cursor = top
        for height in row_h:
            row_top.append(cursor)
            cursor -= height + gap

        spawned = 0
        for index, class_id in enumerate(picks):
            col = index % ncols
            row = index // ncols
            size = sizes[index]
            sx, sy, sz = size

            x = (col_left[col] + col_w[col] - sx / 2.0 if col == 0
                 else col_left[col] + sx / 2.0)
            y = (row_top[row] - row_h[row] + sy / 2.0 if row == 0
                 else row_top[row] - sy / 2.0)

            name = f'sort_obj_{index}'
            rgba = self._randomized_rgba(class_id)
            pose = [float(x), float(y), float(sz / 2.0 + clearance), 0.0, 0.0, 0.0]

            sdf = self._object_sdf(
                name, self.class_table.shape_of(class_id), size, rgba, pose
            )
            if not self._spawn_model(name, sdf, pose):
                continue

            self._objects[name] = {
                'class_id': class_id,
                'class_name': self.class_table.name_of(class_id),
                'shape': self.class_table.shape_of(class_id),
                'size': size,
                'placement_zone': self.class_table.placement_zone_of(class_id),
                'spawn_pose': pose,
            }
            spawned += 1

        self.get_logger().info(
            f'已按紧凑网格生成 {spawned} 个待分拣物体（边界间距 {gap * 1000:.1f}mm）'
        )
        return spawned

    # ==================================================================
    # 服务回调
    # ==================================================================
    def on_reset_scene(self, request, response):
        # 非阻塞获取：避免服务回调长期占用 executor 线程
        if not self._setup_lock.acquire(blocking=False):
            response.success = False
            response.message = '场景正在装配中，请稍后重试'
            response.spawned_objects = 0
            return response

        try:
            self.get_logger().info(
                f'重置场景: n_objects={request.n_objects}, '
                f'randomize_class={request.randomize_class}, '
                f'randomize_pose={request.randomize_pose}, '
                f'randomize_scene={request.randomize_scene}'
            )
            self.ensure_bins()
            if request.randomize_scene:
                self._randomize_camera()
                self._randomize_light()

            count = int(request.n_objects) if request.n_objects > 0 else int(
                self.scene_cfg.get('n_objects', 4)
            )
            try:
                spawned = self._spawn_objects(
                    count, bool(request.randomize_class), bool(request.randomize_pose)
                )
            except Exception as exc:  # 服务调用异常不应导致节点崩溃
                self.get_logger().error(f'生成物体异常: {exc}')
                response.success = False
                response.message = str(exc)
                response.spawned_objects = 0
                return response

            if spawned > 0:
                self._initial_objects_done = True
            response.success = spawned > 0
            response.spawned_objects = spawned
            response.message = f'已生成 {spawned} 个物体'
            return response
        finally:
            self._setup_lock.release()

    # ==================================================================
    # 真值与 TF 发布
    # ==================================================================
    def on_pose_tf(self, msg: TFMessage):
        """缓存各待分拣物体的当前位姿（用于真值发布与评估）。"""
        for transform in msg.transforms:
            model_name = transform.child_frame_id.split('/')[0]
            if not model_name:
                continue
            self._known_models.add(model_name)
            if model_name not in self._objects:
                continue
            pose = Pose()
            pose.position.x = float(transform.transform.translation.x)
            pose.position.y = float(transform.transform.translation.y)
            pose.position.z = float(transform.transform.translation.z)
            pose.orientation = transform.transform.rotation
            self._model_poses[model_name] = pose

    def publish_state(self):
        self._publish_camera_tf()

        array = ObjectStateArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.system_config['frames']['world']

        for name, meta in self._objects.items():
            pose = self._model_poses.get(name)
            if pose is None:
                pose = Pose()
                spawn = meta['spawn_pose']
                pose.position.x, pose.position.y, pose.position.z = spawn[0], spawn[1], spawn[2]
                quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, spawn[5])
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = [float(v) for v in quaternion]

            state = ObjectState()
            state.header = array.header
            state.class_id = int(meta['class_id'])
            state.class_name = meta['class_name']
            state.object_name = name
            state.shape = {'box': 0, 'cylinder': 1, 'sphere': 2}.get(meta['shape'], 0)
            state.pose = pose
            state.size.x, state.size.y, state.size.z = [float(v) for v in meta['size']]
            state.is_grasped = False
            state.placement_zone = meta['placement_zone']
            array.objects.append(state)

        self.pub_ground_truth.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    # 线程数需大于"阻塞式调用数"：高频位姿回调已隔离到独立回调组，
    # 这里再留出余量给定时器与服务回调
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