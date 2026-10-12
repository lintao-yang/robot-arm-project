"""把项目 .venv 的依赖按节点暴露给 launch 启动的 ROS 节点。

背景（本项目实测结论）：
    * 项目 Python 依赖（torch / scikit-learn）装在 ``<workspace>/.venv``；
    * ROS 安装出的可执行脚本 shebang 指向 ``/usr/bin/python3``，该系统解释器
      **没有** torch / scikit-learn；
    * 但**不能**全局注入 venv 的 site-packages：venv 的 numpy 1.26 已移除
      ``np.float`` 等旧别名，会让 MoveIt 等仍依赖旧 numpy 的系统组件直接启动失败
      （实测：全局注入后 move_group 报 ``module 'numpy' has no attribute 'float'``）。

因此这里提供按节点注入的工具：只给确实需要 venv 依赖的节点（点云预处理、
GraspNet 推理）设置 ``additional_env``，MoveIt / 控制等节点保持不变。
"""

import os

from ament_index_python.packages import get_package_share_directory


def _workspace_dir() -> str:
    workspace = os.environ.get('WORKSPACE_DIR')
    if workspace:
        return workspace
    # install/<pkg>/share/<pkg> 向上 4 级即工作空间根目录
    bringup = get_package_share_directory('sorting_bringup')
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(bringup))))


def venv_site_packages() -> str:
    return os.path.join(_workspace_dir(), '.venv', 'lib', 'python3.10', 'site-packages')


def venv_pythonpath() -> str:
    """返回可直接用于 ``Node(additional_env={'PYTHONPATH': ...})`` 的 PYTHONPATH 值。"""
    existing = os.environ.get('PYTHONPATH', '')
    return venv_site_packages() + (os.pathsep + existing if existing else '')