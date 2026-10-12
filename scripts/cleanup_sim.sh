#!/bin/bash
# 清理 WSL 中残留的仿真/ROS 进程（僵尸实例会导致控制器加载竞态与话题争抢）。
# 注意：
#   - 模式用 [x] 括号技巧避免 pgrep 匹配到本脚本自身
#   - 按 PID 显式 kill -9（SIGKILL 不触发旧 acceptance_run.sh 的 EXIT trap，
#     避免旧任务收尾时误杀新启动的进程）
#   - 不要用 pkill -f：PowerShell -> WSL 转义链会破坏模式串
# 注意 install/sorting_ 这一项：ROS 安装出的控制台脚本节点（如 pointcloud_preprocessor、
# graspnet_planner_node）命令行里既没有 "ros2" 也没有 "--ros-args"，只按原模式会漏杀，
# 残留节点会与新实例争抢同名话题/节点，造成 DDS 混乱（曾致验收整轮无抓取）。
PATTERN='[g]azebo|[g]z sim|[r]os-args|[r]os2|[a]cceptance_run|[i]nstall/sorting_|[p]arameter_bridge|[m]ove_group|[c]ontroller_manager|[s]pawner|[s]cene_manager|[g]z_ros2_control'

echo '=== before ==='
pgrep -af "$PATTERN" || echo '(none)'

PIDS=$(pgrep -f "$PATTERN" | grep -vx "${CLEANUP_EXCLUDE:-0}" | tr '\n' ' ')
if [ -n "${PIDS// /}" ]; then
    echo "killing PIDs: $PIDS"
    kill -9 $PIDS 2>/dev/null
    sleep 2
else
    echo 'nothing to kill'
fi

# 清理 FastDDS 共享内存残留：kill -9 杀掉的进程不会释放 /dev/shm/fastrtps_* 段，
# 残留段会导致新启动系统的 DDS 服务/话题投递不可靠（请求发出但响应丢失）。
SHM_COUNT=$(ls -A /dev/shm/ 2>/dev/null | grep -c fastrtps)
if [ "$SHM_COUNT" -gt 0 ]; then
    echo "removing $SHM_COUNT stale fastrtps shm segments"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
fi

echo '=== after ==='
REMAIN=$(pgrep -f "$PATTERN" | tr '\n' ' ')
if [ -n "${REMAIN// /}" ]; then
    echo "STILL REMAINING: $REMAIN"
    pgrep -af "$PATTERN"
    exit 1
fi
echo 'clean: 0 matching processes'
