"""把处理后的 robot_description 以普通 QoS 周期重发，供 Web 操作台加载 URDF。

背景：ROS 2 中 robot_description 由 robot_state_publisher 以瞬态本地（transient
local）QoS 发布，rosbridge 默认 VOLATILE 订阅拿不到历史消息；rosapi 参数接口
又要求 "节点名:参数名" 形式且与本项目的 xacro 处理链不兼容。因此直接复用
sorting_simulation.urdf_utils 的处理链生成 URDF 文本，以 0.5Hz 周期重发到
/web_ui/robot_description（普通 QoS），浏览器端订阅即可随时获取。
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from sorting_simulation.urdf_utils import load_robot_description


class UrdfRelay(Node):
    def __init__(self):
        super().__init__('urdf_relay')
        share = get_package_share_directory('sorting_simulation')
        xacro_file = os.path.join(share, 'urdf', 'panda_gz.urdf.xacro')
        self._urdf = load_robot_description(xacro_file)
        self._pub = self.create_publisher(String, '/web_ui/robot_description', 10)
        self.create_timer(2.0, self._tick)
        self._tick()
        self.get_logger().info('URDF relay 就绪：/web_ui/robot_description (0.5Hz)')

    def _tick(self):
        msg = String()
        msg.data = self._urdf
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = UrdfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
