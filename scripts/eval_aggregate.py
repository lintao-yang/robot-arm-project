"""聚合批量评估运行结果：解析每轮 launch.log 的 [EVAL] 行，计算并汇总三层指标。

用法:
    python3 scripts/eval_aggregate.py [--root logs/eval_runs] [--out evaluation]

对每轮 run_XX 计算:
    端到端抓取成功率 = picked_success / attempted
    规划成功率       = plan_successes / plan_requests
    空抓率           = empty_grasps / close_attempts
    放置成功率       = picked_success / grasp_confirmed
再对多轮取均值与标准差，输出 evaluation/runs_summary.json 与打印 Markdown 表。
"""

import argparse
import glob
import json
import os
import re
from statistics import mean, pstdev

EVAL_RE = re.compile(r'\[EVAL\]\s*(\{.*\})')


def parse_run(run_dir):
    """解析单轮目录：取 launch.log 里最后一行 [EVAL]，合并 statistics 汇总。"""
    log_path = os.path.join(run_dir, 'launch.log')
    evals = []
    if os.path.isfile(log_path):
        with open(log_path, 'r', encoding='utf-8', errors='ignore') as handle:
            for line in handle:
                m = EVAL_RE.search(line)
                if m:
                    try:
                        evals.append(json.loads(m.group(1)))
                    except json.JSONDecodeError:
                        pass
    if not evals:
        return None
    ev = evals[-1]  # 最后一行即任务收尾时的累计值

    rec = {'run': os.path.basename(run_dir), 'done': True}
    rec.update(ev)

    summary_path = os.path.join(run_dir, 'summary.json')
    if os.path.isfile(summary_path):
        with open(summary_path, 'r', encoding='utf-8') as handle:
            summary = json.load(handle)
        rec['state'] = summary.get('state', '')
        rec['duration_s'] = summary.get('elapsed', None)
    else:
        rec['state'] = ''

    # 派生指标
    att = rec.get('attempted', 0)
    rec['e2e_pick_success_rate'] = round(rec.get('picked_success', 0) / att, 4) if att else 0.0
    pr = rec.get('plan_requests', 0)
    rec['planning_success_rate'] = round(rec.get('plan_successes', 0) / pr, 4) if pr else 0.0
    ca = rec.get('close_attempts', 0)
    rec['empty_grasp_rate'] = round(rec.get('empty_grasps', 0) / ca, 4) if ca else 0.0
    gc = rec.get('grasp_confirmed', 0)
    rec['place_success_rate'] = round(rec.get('picked_success', 0) / gc, 4) if gc else 0.0
    return rec


def aggregate(records):
    recs = [r for r in records if r]
    if not recs:
        return {}
    metrics = [
        'attempted', 'picked_success', 'plan_requests', 'plan_successes',
        'empty_grasps', 'grasp_confirmed', 'lift_slip', 'drop_plan_fail',
        'e2e_pick_success_rate', 'planning_success_rate',
        'empty_grasp_rate', 'place_success_rate', 'avg_cycle_time',
    ]
    agg = {}
    for key in metrics:
        vals = [float(r[key]) for r in recs if r.get(key) is not None]
        if vals:
            agg[key] = {'mean': round(mean(vals), 4), 'std': round(pstdev(vals), 4),
                        'n': len(vals)}
    # 汇总口径（比率用总量再算一遍，避免小样本均值失真）
    tot_att = sum(r['attempted'] for r in recs)
    tot_pick = sum(r['picked_success'] for r in recs)
    tot_pr = sum(r['plan_requests'] for r in recs)
    tot_ps = sum(r['plan_successes'] for r in recs)
    tot_close = sum(r['close_attempts'] for r in recs)
    tot_empty = sum(r['empty_grasps'] for r in recs)
    tot_conf = sum(r['grasp_confirmed'] for r in recs)
    agg['overall_pooled'] = {
        'attempted': tot_att,
        'picked_success': tot_pick,
        'e2e_pick_success_rate': round(tot_pick / tot_att, 4) if tot_att else 0.0,
        'plan_requests': tot_pr,
        'plan_successes': tot_ps,
        'planning_success_rate': round(tot_ps / tot_pr, 4) if tot_pr else 0.0,
        'close_attempts': tot_close,
        'empty_grasps': tot_empty,
        'empty_grasp_rate': round(tot_empty / tot_close, 4) if tot_close else 0.0,
        'grasp_confirmed': tot_conf,
        'place_success_rate': round(tot_pick / tot_conf, 4) if tot_conf else 0.0,
    }
    agg['n_runs'] = len(recs)
    return agg


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default=os.path.join(here, 'logs/eval_runs'))
    parser.add_argument('--out', default=os.path.join(here, 'evaluation'))
    args = parser.parse_args()

    run_dirs = sorted(glob.glob(os.path.join(args.root, 'run_*')))
    records = [parse_run(d) for d in run_dirs]
    ok = [r for r in records if r]
    agg = aggregate(records)

    os.makedirs(args.out, exist_ok=True)
    payload = {'root': args.root, 'runs': records, 'aggregate': agg}
    with open(os.path.join(args.out, 'runs_summary.json'), 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)

    print(f'解析到 {len(ok)}/{len(run_dirs)} 轮有效结果')
    print(f'{"run":8} {"att":>4} {"pick":>5} {"planR":>6} {"planS":>6} '
          f'{"E2E":>7} {"PlanSR":>7} {"Empty":>7} {"PlaceSR":>8}')
    for r in records:
        if not r:
            print(f'{os.path.basename(str(r)):8}  <无 [EVAL] 行>')
            continue
        print(f'{r["run"]:8} {r["attempted"]:>4} {r["picked_success"]:>5} '
              f'{r["plan_requests"]:>6} {r["plan_successes"]:>6} '
              f'{r["e2e_pick_success_rate"]:>7.3f} {r["planning_success_rate"]:>7.3f} '
              f'{r["empty_grasp_rate"]:>7.3f} {r["place_success_rate"]:>8.3f}')
    if agg.get('overall_pooled'):
        p = agg['overall_pooled']
        print('\n[汇总 pooled] '
              f'E2E={p["e2e_pick_success_rate"]:.3f} '
              f'PlanSR={p["planning_success_rate"]:.3f} '
              f'Empty={p["empty_grasp_rate"]:.3f} '
              f'PlaceSR={p["place_success_rate"]:.3f}')


if __name__ == '__main__':
    main()
