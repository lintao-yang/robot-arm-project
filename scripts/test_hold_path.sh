#!/bin/bash
# 夹持指令通路探针：验证 forward command 控制器是否真的持续执行最后一次指令。
# 方法：手动下发 -30N 保持力矩，观察停止发指令后手指位置是否漂移。
#   - 若不漂移   => 指令在硬件接口层保持有效，滑落是物理（力/摩擦/几何）问题
#   - 若明显漂移 => 指令丢失属实，需更高频刷新或换控制器类型
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

python3 - <<'EOF'
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = rclpy.create_node('hold_path_probe')
positions = {}

def cb(msg):
    for name, pos in zip(msg.name, msg.position):
        if 'finger' in name:
            positions[name] = pos

node.create_subscription(JointState, '/joint_states', cb, 10)
pub = node.create_publisher(Float64MultiArray, '/panda_hand_controller/commands', 10)

def spin_for(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

def show(tag):
    print(tag, {k: round(v, 4) for k, v in positions.items()}, flush=True)

def cmd(value):
    msg = Float64MultiArray()
    msg.data = [value, value]
    pub.publish(msg)

spin_for(2.0)
show('[1] 初始位置')
cmd(-30.0)
spin_for(1.0)
show('[2] 下发-30N后1s')
spin_for(3.0)
show('[3] 停止发指令3s后（关键：检验指令是否保持）')
cmd(-30.0)
spin_for(1.0)
show('[4] 再次下发-30N后1s')
cmd(6.0)
spin_for(1.5)
show('[5] 张开恢复')
node.destroy_node()
rclpy.shutdown()
EOF
