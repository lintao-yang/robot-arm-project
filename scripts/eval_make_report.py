"""生成 Markdown 评估报告：合并感知层（YOLO）与系统层（10 轮抓取）指标。

用法:
    python3 scripts/eval_make_report.py [--runs evaluation/runs_summary.json]
                                        [--yolo evaluation/yolo_metrics.json]
                                        [--out evaluation/GRASP_EVALUATION_REPORT.md]
"""

import argparse
import json
import os
from datetime import datetime


def load(path):
    if os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as handle:
            return json.load(handle)
    return None


def pct(value):
    return f'{value * 100:.1f}%'


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', default=os.path.join(here, 'evaluation/runs_summary.json'))
    parser.add_argument('--yolo', default=os.path.join(here, 'evaluation/yolo_metrics.json'))
    parser.add_argument('--out', default=os.path.join(here, 'evaluation/GRASP_EVALUATION_REPORT.md'))
    args = parser.parse_args()

    runs = load(args.runs) or {}
    yolo = load(args.yolo) or {}
    recs = [r for r in runs.get('runs', []) if r]
    agg = runs.get('aggregate', {})
    pooled = agg.get('overall_pooled', {})

    lines = []
    add = lines.append
    add('# 机械臂仿真抓取系统 —— 分层指标评估报告\n')
    add(f'生成时间：{datetime.now().strftime("%Y-%m-%d %H:%M")}\n')
    add('> 系统：ROS 2 Humble + MoveIt 2 + Gazebo Sim (Fortress)，Franka Panda 两指夹爪，')
    add('> 感知链路：YOLOv8-seg 2D 实例分割 + 点云 DBSCAN 聚类/拆分 + 预训练 GraspNet 6DoF 抓取检测。\n')

    # ---------------- 1. 实验设置 ----------------
    add('## 1. 实验设置\n')
    add(f'- 系统层采集：headless 完整系统，`continuous: true` 连续抓取，每轮跑到任务 DONE 或 480s 上限；'
        f'共 **{agg.get("n_runs", len(recs))} 轮**有效运行。')
    add('- 场景摆放：每轮全新启动仿真，采用**固定初始布局**（2×2 紧凑网格，边界间距 20mm），'
        '以隔离物理/时序噪声、保证指标可复现可对比。')
    add('- 感知层评估：离线在合成验证集（29 张 / 116 个实例，含真值多边形）上评估 YOLOv8-seg；'
        '每张图均经随机化摆放采集，与在线场景同源。\n')

    # ---------------- 2. 感知层 ----------------
    add('## 2. 感知层（YOLOv8-seg 2D 实例分割）\n')
    if yolo:
        add('| 指标 | 数值 | 说明 |')
        add('|---|---|---|')
        add(f'| **mAP@0.5 (Mask)** | **{yolo.get("mAP50_mask")}** | 掩码平均精度均值（IoU 阈值 0.5） |')
        add(f'| mAP@0.5:0.95 (Mask) | {yolo.get("mAP50_95_mask")} | 掩码 mAP（IoU 0.5:0.95 平均） |')
        add(f'| Mask Precision | {yolo.get("precision_mask")} | 掩码精确率 |')
        add(f'| Mask Recall | {yolo.get("recall_mask")} | 掩码召回率 |')
        add(f'| **平均 Mask IoU（匹配实例）** | **{yolo.get("mean_iou_matched")}** | '
            f'预测掩码与真值掩码的交并比均值（IoU≥0.5 贪心匹配） |')
        add(f'| 实例检出率 | {pct(yolo.get("detection_rate", 0))} | '
            f'命中实例 {yolo.get("n_matched")}/{yolo.get("n_gt")} |')
        add('')
        pci = yolo.get('per_class_mean_iou', {})
        names = yolo.get('class_names', {})
        if pci:
            add('各类别平均 Mask IoU：')
            add('')
            add('| 类别 | red_cube | blue_cylinder | green_bar | yellow_box |')
            add('|---|---|---|---|---|')
            row = ' | '.join(str(pci.get(k, '-')) for k in ['0', '1', '2', '3'])
            add(f'| 平均 IoU | {row} |')
            add('')
        add('**结论**：合成场景中物体为纯色几何体，YOLOv8-seg 的 mask mAP@0.5 达 '
            f'{yolo.get("mAP50_mask")}；在物体两两紧邻（DBSCAN 会将其并为一团）的场景下，'
            f'分割掩码的平均 IoU 仍达 {yolo.get("mean_iou_matched")}，可有效在 2D 上把粘连实例切开，'
            '为下游点云按类别拆分粘连簇提供可靠依据。\n')

    # ---------------- 3. 规划与执行层 ----------------
    add('## 3. 规划与执行层（GraspNet + MoveIt 2）\n')
    add('| 指标 | 数值（跨轮均值） | 汇总口径（总量比） | 说明 |')
    add('|---|---|---|---|')
    pr = agg.get('planning_success_rate', {})
    er = agg.get('empty_grasp_rate', {})
    add(f'| **规划成功率** | {pr.get("mean", "-")} ± {pr.get("std", "-")} | '
        f'**{pct(pooled.get("planning_success_rate", 0))}** | '
        f'MoveIt 成功规划并执行的比例（{pooled.get("plan_successes")}/{pooled.get("plan_requests")} 次请求） |')
    add(f'| **空抓率** | {er.get("mean", "-")} ± {er.get("std", "-")} | '
        f'**{pct(pooled.get("empty_grasp_rate", 0))}** | '
        f'夹爪闭合到底未夹住 / 闭合尝试（{pooled.get("empty_grasps")}/{pooled.get("close_attempts")}） |')
    add('')
    add('规划成功率按"每次 MoveIt 运动请求（位姿目标 / 直线插补 / 关节目标）成功实现"统计；'
        '空抓率按"下压到位后闭合夹爪、但未夹住物体"统计，直接反映抓取深度/偏心问题。\n')

    # ---------------- 4. 系统层 ----------------
    add('## 4. 系统层（端到端闭环）\n')
    add('| 指标 | 数值（跨轮均值） | 汇总口径（总量比） | 说明 |')
    add('|---|---|---|---|')
    e2e = agg.get('e2e_pick_success_rate', {})
    pl = agg.get('place_success_rate', {})
    add(f'| **综合抓取成功率 (E2E)** | {e2e.get("mean", "-")} ± {e2e.get("std", "-")} | '
        f'**{pct(pooled.get("e2e_pick_success_rate", 0))}** | '
        f'从感知到夹起并搬运成功 / 尝试次数（{pooled.get("picked_success")}/{pooled.get("attempted")}） |')
    add(f'| **放置成功率** | {pl.get("mean", "-")} ± {pl.get("std", "-")} | '
        f'**{pct(pooled.get("place_success_rate", 0))}** | '
        f'夹住并成功搬运投放 / 抓取确认次数（{pooled.get("picked_success")}/{pooled.get("grasp_confirmed")}） |')
    lift = agg.get('lift_slip', {})
    drop = agg.get('drop_plan_fail', {})
    add(f'| 抬升掉落（lift_slip） | {lift.get("mean", "-")} | 合计 {sum(r.get("lift_slip", 0) for r in recs)} | '
        '已夹住但抬升失败 |')
    add(f'| 投放点规划失败 | {drop.get("mean", "-")} | 合计 {sum(r.get("drop_plan_fail", 0) for r in recs)} | '
        '已夹住但投放点上方规划失败 |')
    add('')
    cyc = agg.get('avg_cycle_time', {})
    if cyc:
        add(f'- 平均单次抓取周期：**{cyc.get("mean")} s**（跨轮均值 ± {cyc.get("std")}）\n')

    # ---------------- 5. 逐轮明细 ----------------
    add('## 5. 逐轮明细\n')
    add('| 轮次 | 结束状态 | 时长(s) | 尝试 | 成功搬走 | 规划请求/成功 | 规划成功率 | '
        '闭合尝试 | 空抓 | 空抓率 | 抓取确认 | 放置成功率 | E2E 成功率 |')
    add('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in runs.get('runs', []):
        if not r:
            continue
        add(f'| {r.get("run")} | {r.get("state", "")} | '
            f'{r.get("duration_s", "") if r.get("duration_s") is not None else ""} | '
            f'{r.get("attempted")} | {r.get("picked_success")} | '
            f'{r.get("plan_requests")}/{r.get("plan_successes")} | {r.get("planning_success_rate")} | '
            f'{r.get("close_attempts")} | {r.get("empty_grasps")} | {r.get("empty_grasp_rate")} | '
            f'{r.get("grasp_confirmed")} | {r.get("place_success_rate")} | {r.get("e2e_pick_success_rate")} |')
    add('')

    # ---------------- 6. 口径说明 ----------------
    add('## 6. 指标口径与测量方式\n')
    add('| 指标 | 定义 | 采集方式 |')
    add('|---|---|---|')
    add('| mAP@0.5 (Mask) | 掩码 AP 在 IoU=0.5 下的均值 | ultralytics `val()`，与训练日志同口径 |')
    add('| 平均 Mask IoU | 命中实例（IoU≥0.5 贪心匹配）预测掩码与真值掩码 IoU 均值 | 自实现逐实例匹配 |')
    add('| 规划成功率 | MoveIt 运动请求成功数 / 请求总数 | `MoveGroupClient` 埋点计数 |')
    add('| 空抓率 | 闭合到底未夹住次数 / 闭合尝试次数 | 抓取执行节点埋点计数 |')
    add('| 综合抓取成功率 | 成功夹起并搬运放置次数 / 尝试次数 | 抓取执行节点计数 |')
    add('| 放置成功率 | 成功搬运投放次数 / 抓取确认（夹住）次数 | 抓取执行节点计数 |')
    add('')

    with open(args.out, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    print(f'报告已生成: {args.out}')


if __name__ == '__main__':
    main()
