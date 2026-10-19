"""离线评估 YOLOv8-seg（感知层指标）：mask mAP@0.5、mAP@0.5:0.95 与平均掩码 IoU。

用法:
    .venv/bin/python scripts/eval_yolo_seg.py [--weights ...] [--data ...] [--conf 0.25]

输出 JSON 到 stdout，含:
    mAP50_mask / mAP50_95_mask / precision_mask / recall_mask
    mean_iou / mean_iou_matched / detection_rate / n_gt / n_matched / per_class

说明:
    - mAP 直接采用 ultralytics 官方 val() 结果（与训练日志同口径）。
    - 平均 IoU 为自实现：对每张验证图，按类别做 IoU 降序贪心匹配（IoU>=0.5 视为命中），
      统计命中实例的预测掩码与 GT 掩码的 IoU 均值——这正是"紧邻场景下分割是否切开"的量度。
"""

import argparse
import json
import os
import glob

import cv2
import numpy as np
from ultralytics import YOLO

IOU_MATCH_THRESHOLD = 0.5


def load_gt_masks(label_path, w, h):
    """读取 YOLOv8-seg 多边形标签，栅格化为二值掩码。返回 [(class_id, mask)]。"""
    out = []
    if not os.path.isfile(label_path):
        return out
    with open(label_path, 'r', encoding='utf-8') as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 7:  # class + 至少 3 个点
                continue
            cid = int(float(parts[0]))
            coords = np.array([float(v) for v in parts[1:]], dtype=np.float32).reshape(-1, 2)
            coords[:, 0] *= w
            coords[:, 1] *= h
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(mask, [coords.astype(np.int32)], 1)
            out.append((cid, mask))
    return out


def mask_iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter) / float(union) if union else 0.0


def greedy_match(pred_masks, gt_masks):
    """同一类别内按 IoU 降序贪心匹配，返回命中对的 IoU 列表。"""
    pairs = []
    for pi, pm in enumerate(pred_masks):
        for gi, gm in enumerate(gt_masks):
            pairs.append((mask_iou(pm, gm), pi, gi))
    pairs.sort(key=lambda t: -t[0])
    used_p, used_g, matched = set(), set(), []
    for iou, pi, gi in pairs:
        if iou < IOU_MATCH_THRESHOLD:
            break
        if pi in used_p or gi in used_g:
            continue
        used_p.add(pi)
        used_g.add(gi)
        matched.append(iou)
    return matched


def main():
    parser = argparse.ArgumentParser()
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument('--weights', default=os.path.join(
        here, 'runs/segment/sort_objects/weights/best.pt'))
    parser.add_argument('--data', default=os.path.join(here, 'datasets/yolo_seg/data.yaml'))
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()

    model = YOLO(args.weights)

    # ---- 1) 官方 val()：mask mAP ----
    val = model.val(data=args.data, split='val', imgsz=args.imgsz,
                    device=args.device, verbose=False)
    result = {
        'weights': args.weights,
        'data': args.data,
        'mAP50_mask': round(float(val.seg.map50), 4),
        'mAP50_95_mask': round(float(val.seg.map), 4),
        'precision_mask': round(float(val.seg.mp), 4),
        'recall_mask': round(float(val.seg.mr), 4),
        'mAP50_box': round(float(val.box.map50), 4),
    }

    # ---- 2) 自实现掩码 IoU（逐实例贪心匹配）----
    val_images = sorted(glob.glob(os.path.join(os.path.dirname(args.data),
                                               'images/val/*.jpg')))
    val_labels_dir = os.path.join(os.path.dirname(args.data), 'labels/val')
    all_ious, per_class = [], {}
    n_gt_total, n_matched_total = 0, 0
    for img_path in val_images:
        bgr = cv2.imread(img_path)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        stem = os.path.splitext(os.path.basename(img_path))[0]
        gt = load_gt_masks(os.path.join(val_labels_dir, stem + '.txt'), w, h)
        n_gt_total += len(gt)

        res = model.predict(source=bgr, conf=args.conf, imgsz=args.imgsz,
                            device=args.device, verbose=False)[0]
        preds = []  # (class_id, mask bool)
        if res.masks is not None and res.boxes is not None:
            cls = res.boxes.cls.detach().cpu().numpy().astype(int)
            masks = res.masks.data.detach().cpu().numpy()
            for cid, m in zip(cls, masks):
                if m.shape != (h, w):
                    m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                preds.append((int(cid), m > 0.5))

        classes = set(c for c, _ in gt) | set(c for c, _ in preds)
        for cid in classes:
            pm = [m for c, m in preds if c == cid]
            gm = [m for c, m in gt if c == cid]
            matched = greedy_match(pm, gm)
            all_ious.extend(matched)
            per_class.setdefault(cid, []).extend(matched)
            n_matched_total += len(matched)

    result['n_gt'] = n_gt_total
    result['n_matched'] = n_matched_total
    result['detection_rate'] = round(n_matched_total / n_gt_total, 4) if n_gt_total else 0.0
    result['mean_iou_matched'] = round(float(np.mean(all_ious)), 4) if all_ious else 0.0
    result['per_class_mean_iou'] = {
        str(cid): round(float(np.mean(v)), 4) for cid, v in sorted(per_class.items())
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
