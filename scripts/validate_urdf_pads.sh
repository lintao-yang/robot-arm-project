#!/bin/bash
# 校验 panda_gz.urdf.xacro 展开结果：V 槽垫片与手指 effort 上限
# （dump_urdf 输出为单行 XML，一律用 grep -o 按出现次数统计）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

OUT=/tmp/robot_expanded.urdf
ros2 run sorting_simulation dump_urdf > "$OUT" 2>/tmp/dump_urdf_err.log
if [ ! -s "$OUT" ]; then
  echo 'FAIL: dump_urdf 输出为空'; cat /tmp/dump_urdf_err.log; exit 1
fi

echo "垫片 link 定义数: $(grep -o 'link name="panda_[a-z]*finger_pad"' "$OUT" | sort -u | wc -l)  (期望 2)"
echo "垫片固定关节数: $(grep -o 'joint name="panda_[a-z]*finger_pad_joint"' "$OUT" | sort -u | wc -l)  (期望 2)"
echo "V槽碰撞面数: $(grep -o 'collision name="vgroove_face' "$OUT" | wc -l)  (期望 4)"
echo "preserveFixedJoint: $(grep -o 'preserveFixedJoint>true' "$OUT" | wc -l)  (期望 2)"
echo "手指 effort=40.0: $(grep -o 'effort="40.0"' "$OUT" | wc -l)  (期望 2)"
echo "link 重名检查: $(grep -o 'link name="[a-zA-Z0-9_]*"' "$OUT" | sort | uniq -d | wc -l)  (期望 0)"
