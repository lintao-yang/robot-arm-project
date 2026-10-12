"""相机外参与坐标系换算（全项目唯一真值源）。

为什么需要单独一个模块：相机安装位姿（system.yaml 的 ``camera.mount_pose``）必须与
``worlds/sorting_world_sim.sdf`` 里相机模型的 ``<pose>`` 一致，而**使用它的有两处**
（``pointcloud_preprocessor`` 与 ``graspnet_planner_node``）。此前两处各写了一份，
且都只取了 ``pitch``（``r = Ry(pitch)``）——只在"相机光轴朝正下方"时成立。相机一旦
改成侧上方俯视，外参会全错、点云三维定位出现系统性偏差。这里统一为完整 RPY。
"""

import numpy as np

# 相机 body 系（x 前 / y 左 / z 上）-> 光学系（z 前 / x 右 / y 下）
# Gazebo 的相机传感器沿 body 系 +x 成像；GraspNet 按光学系训练，必须转到光学系。
R_BODY_TO_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF/URDF 约定的 RPY（固定轴 x-y-z 外旋）转旋转矩阵：Rz(yaw)·Ry(pitch)·Rx(roll)。

    ``pitch = +pi/2`` 时 body 的 +x（成像方向）被转到 world 的 -z，即光轴朝正下方。
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def camera_extrinsics(cfg: dict):
    """由配置推出 ``(t_world_cam, r_world_optical)``。

    用法：``world_pt = optical_pt @ r_world_optical.T + t_world_cam``。
    """
    mount = cfg['camera']['mount_pose']
    t_world_cam = np.array([mount['x'], mount['y'], mount['z']], dtype=float)
    r_world_body = rpy_to_matrix(
        float(mount.get('roll', 0.0)),
        float(mount.get('pitch', 0.0)),
        float(mount.get('yaw', 0.0)),
    )
    return t_world_cam, r_world_body @ R_BODY_TO_OPTICAL