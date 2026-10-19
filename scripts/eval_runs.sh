#!/bin/bash
# 批量评估运行：headless 启动完整分拣系统，连续抓取直到 DONE 或单轮超时，逐轮落盘。
#   用法: bash scripts/eval_runs.sh [轮数=10] [单轮上限秒=900]
# 有效性门槛（任一不满足即判本轮无效并自动重试，不计入轮数）：
#   * controller_manager / move_group / 相机点云 / 工作区点云 全部就绪
#   * 场景装配完成（launch.log 出现"场景装配完成"）
#   * 抓取执行节点就绪（launch.log 出现"执行节点就绪"）
#   * start_sorting 返回 success=True
#   * 控制器未出现"配置失败（已重试 5 次）"、且至少产生一次抓取尝试（[EVAL]）
# 每轮产物: logs/eval_runs/run_XX/{launch.log, statistics.csv, summary.json, task_status.log}
# 注意：不能使用 set -u（ROS setup.bash 会引用未定义变量）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
# shellcheck source=env.sh
source "$WORKSPACE_DIR/scripts/env.sh"
export DISPLAY="${DISPLAY:-:0}"

N="${1:-10}"
LIMIT="${2:-900}"
MAX_ATTEMPTS=$((N * 4))
OUT_ROOT="$WORKSPACE_DIR/logs/eval_runs"
mkdir -p "$OUT_ROOT"
cd "$WORKSPACE_DIR"

valid=0
attempt=0
while [ "$valid" -lt "$N" ] && [ "$attempt" -lt "$MAX_ATTEMPTS" ]; do
  attempt=$((attempt + 1))
  RUN_DIR="$OUT_ROOT/run_$(printf '%02d' $((valid + 1)))"
  rm -rf "$RUN_DIR"; mkdir -p "$RUN_DIR"
  LOG="$RUN_DIR/launch.log"
  echo "===== 有效轮 $((valid + 1))/$N（第 $attempt 次尝试，$(date '+%H:%M:%S')）====="

  # 启动前清场：僵尸进程与 FastDDS 共享内存残留会导致控制器竞态与 DDS 投递异常
  CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 || true
  sleep 2

  PREV_RUN=$(python3 -c "import json;print(json.load(open('logs/latest.json')).get('run_dir',''))" 2>/dev/null || true)

  ros2 launch sorting_bringup sorting_system.launch.py gui:=false rviz:=false > "$LOG" 2>&1 &
  LPID=$!

  echo '[ready] 等待系统就绪（服务/话题 + 场景装配 + 执行节点引导）...'
  READY=0
  DEADLINE=$((SECONDS + 300))
  while [ "$SECONDS" -lt "$DEADLINE" ]; do
    SVCS=$(ros2 service list 2>/dev/null)
    if echo "$SVCS" | grep -q '/controller_manager/list_controllers' \
       && echo "$SVCS" | grep -q '/apply_planning_scene' \
       && timeout 5 ros2 topic hz /camera/points 2>/dev/null | grep -q 'average rate' \
       && timeout 5 ros2 topic hz /sorting/workspace_cloud 2>/dev/null | grep -q 'average rate' \
       && grep -qa '场景装配完成' "$LOG" \
       && grep -qa '执行节点就绪' "$LOG"; then
      READY=1; echo '[ready] 全部就绪'; break
    fi
    sleep 3
  done
  [ "$READY" = 1 ] || echo '[ready] 未在时限内全部就绪'

  # 本轮 metrics_logger 输出目录（须与本轮之前不同）
  METRICS_DIR=""
  for _ in $(seq 1 20); do
    CUR=$(python3 -c "import json;print(json.load(open('logs/latest.json')).get('run_dir',''))" 2>/dev/null || true)
    if [ -n "$CUR" ] && [ "$CUR" != "$PREV_RUN" ]; then METRICS_DIR="$CUR"; break; fi
    sleep 2
  done
  echo "[info] metrics 目录: ${METRICS_DIR:-<未知>}"

  if [ "$READY" != 1 ]; then
    echo '[invalid] 系统未完全就绪 -> 本轮作废并重试'
    kill "$LPID" 2>/dev/null; sleep 3
    if [ -n "$METRICS_DIR" ] && [ -d "$METRICS_DIR" ]; then cp -a "$METRICS_DIR/." "$RUN_DIR/" 2>/dev/null || true; fi
    CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 || true
    sleep 2
    continue
  fi

  echo '[start] 触发连续抓取'
  RESP=$(ros2 service call /sorting/start_sorting sorting_interfaces/srv/StartSorting \
    "{use_closed_loop: false, continuous: true}" 2>&1)
  echo "$RESP" | tail -2
  if ! echo "$RESP" | grep -q 'success=True'; then
    echo '[invalid] start_sorting 未成功 -> 本轮作废并重试'
    kill "$LPID" 2>/dev/null; sleep 3
    if [ -n "$METRICS_DIR" ] && [ -d "$METRICS_DIR" ]; then cp -a "$METRICS_DIR/." "$RUN_DIR/" 2>/dev/null || true; fi
    CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 || true
    sleep 2
    continue
  fi

  echo "[wait] 等待 DONE（上限 ${LIMIT}s）"
  DEADLINE=$((SECONDS + LIMIT))
  STATE=""
  while [ "$SECONDS" -lt "$DEADLINE" ]; do
    if [ -n "$METRICS_DIR" ] && [ -f "$METRICS_DIR/summary.json" ]; then
      STATE=$(python3 -c "import json;print(json.load(open('$METRICS_DIR/summary.json')).get('state',''))" 2>/dev/null)
      if [ "$STATE" = "DONE" ]; then echo '[wait] 任务 DONE'; break; fi
    fi
    sleep 3
  done
  echo "[wait] 结束状态=${STATE:-<未达 DONE，超时终止>}"

  kill "$LPID" 2>/dev/null
  sleep 3
  if [ -n "$METRICS_DIR" ] && [ -d "$METRICS_DIR" ]; then cp -a "$METRICS_DIR/." "$RUN_DIR/" 2>/dev/null || true; fi
  CLEANUP_EXCLUDE=$$ bash "$SCRIPT_DIR/cleanup_sim.sh" >/dev/null 2>&1 || true
  sleep 2

  if grep -qa '配置失败（已重试 5 次）' "$LOG"; then
    echo '[invalid] 控制器加载失败 -> 本轮作废并重试'
    continue
  fi
  if ! grep -qa '\[EVAL\]' "$LOG"; then
    echo '[invalid] 本轮无任何抓取尝试（无 [EVAL]）-> 作废并重试'
    continue
  fi

  valid=$((valid + 1))
  echo "[done] 有效轮 ${valid}/${N} 产物: $RUN_DIR"
done

echo "===== 结束：有效 ${valid}/${N} 轮，产物目录: $OUT_ROOT ====="
