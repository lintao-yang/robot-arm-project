#!/bin/bash
# 系统级验收测试：headless 启动完整分拣系统，触发连续分拣，收集结果
# 用法: bash scripts/acceptance_run.sh [运行时长秒，默认 420]
# 注意：不能使用 set -u（ROS 的 setup.bash 会引用未定义变量）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

DURATION="${1:-420}"
RUN_LOG=/tmp/sorting_acceptance.log

cd "$WORKSPACE_DIR"
# 启动前清场：僵尸进程与 FastDDS 共享内存残留会导致控制器竞态与 DDS 投递异常
# （CLEANUP_EXCLUDE 防止清理模式误匹配本脚本自身）
CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" || true
sleep 2

ros2 launch sorting_bringup sorting_system.launch.py gui:=false rviz:=false \
  > "$RUN_LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null
  sleep 3
  # Fortress 的进程名是 'ign gazebo' 而非 'gz sim'，统一交给 cleanup_sim.sh 收尾
  # （脱离本会话执行，避免清理模式误杀本脚本自身导致收尾中断）
  nohup bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 &
  true
}
trap cleanup EXIT

echo '[1/4] 等待系统就绪（轮询：controller_manager / move_group / 相机点云，最多 240s）...'
DEADLINE=$((SECONDS + 240))
READY_CTL=0; READY_MG=0; READY_CAM=0; READY_CLOUD=0
while [ "$SECONDS" -lt "$DEADLINE" ]; do
  SVCS=$(ros2 service list 2>/dev/null)
  if [ "$READY_CTL" = 0 ] && echo "$SVCS" | grep -q '/controller_manager/list_controllers'; then
    READY_CTL=1; echo '  - controller_manager 就绪'
  fi
  if [ "$READY_MG" = 0 ] && echo "$SVCS" | grep -q '/apply_planning_scene'; then
    READY_MG=1; echo '  - move_group 就绪'
  fi
  if [ "$READY_CAM" = 0 ] && timeout 5 ros2 topic hz /camera/points 2>/dev/null | grep -q 'average rate'; then
    READY_CAM=1; echo '  - 相机点云在发布'
  fi
  if [ "$READY_CLOUD" = 0 ] && timeout 5 ros2 topic hz /sorting/workspace_cloud 2>/dev/null | grep -q 'average rate'; then
    READY_CLOUD=1; echo '  - 工作区点云在发布'
  fi
  if [ "$READY_CTL" = 1 ] && [ "$READY_MG" = 1 ] && [ "$READY_CAM" = 1 ] && [ "$READY_CLOUD" = 1 ]; then
    echo '  - 系统全部就绪'; break
  fi
  sleep 3
done
if [ "${READY_CTL}${READY_MG}${READY_CAM}${READY_CLOUD}" != "1111" ]; then
  echo "  ! 就绪检查未全部通过 (ctl=$READY_CTL mg=$READY_MG cam=$READY_CAM cloud=$READY_CLOUD)，仍尝试触发"
fi
sleep 5

echo '[2/4] 触发连续抓取...'
ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
  "{use_closed_loop: false, continuous: true}"

echo "[3/4] 运行 ${DURATION}s ..."
sleep "$DURATION"

echo '[4/4] 结果汇总'
echo '--- 抓取过程 ---'
grep -aE '状态\]|候选#|未夹住|不可达|抬升失败|投放点上方规划失败|抓取并搬运完成' "$RUN_LOG" | tail -40
echo '--- 指标快照 ---'
cat "$WORKSPACE_DIR/logs/latest.json" 2>/dev/null || echo '无 latest.json'
