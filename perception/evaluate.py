#!/usr/bin/env python3
"""
Evaluate trained kart detectors where it matters, not just on the pooled val set.

    python perception/evaluate.py --runs RUNS/v1_yolo11n RUNS/v1_yolo26n \
        --dataset DATASETS/kart_v1 --video data/internet_test_footage/driver-pov-different-track.mp4 \
        --roi-frac 0 0 1 0.6 --out RUNS/eval_v1

1. Val metrics per source (manifest.csv from build_dataset.py): a parking-lot
   score and a track score are different questions, and the pooled number
   hides the one we care about.
2. Onboard video: samples frames, runs the exported ONNX through KartDetector
   exactly as the node will (same letterbox, NMS, optional ROI crop that
   keeps our own hood out), and renders contact sheets for a human check.
   There are no labels for these clips, so this is qualitative.
"""

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src", "autonomous_kart"))
from autolabel import render_sheets  # noqa: E402
from autonomous_kart.nodes.opencv_pathfinder.kart_detector import KartDetector  # noqa: E402


def val_by_source(run: str, dataset: str, imgsz: int, device: str) -> dict:
    from ultralytics import YOLO

    dataset = os.path.abspath(dataset)  # Ultralytics resolves list files against `path`
    rows = [r for r in csv.DictReader(open(os.path.join(dataset, "manifest.csv"))) if r["split"] == "val"]
    groups = defaultdict(list)
    for r in rows:
        groups[r["source"]].append(os.path.join(dataset, "images", "val", r["name"] + ".jpg"))
    groups["ALL"] = [p for v in list(groups.values()) for p in v]
    out = {}
    model = YOLO(os.path.join(run, "weights", "best.pt"))
    for src, paths in groups.items():
        lst = os.path.join(dataset, f"val_{src}.txt")
        with open(lst, "w") as f:
            f.write("\n".join(paths) + "\n")
        yml = os.path.join(dataset, f"val_{src}.yaml")
        with open(yml, "w") as f:
            f.write(f"path: {os.path.abspath(dataset)}\ntrain: {lst}\nval: {lst}\nnames: {{0: kart}}\n")
        m = model.val(data=yml, imgsz=imgsz, device=device, batch=16, plots=False, verbose=False,
                      project=os.path.join(run, "eval"), name=src, exist_ok=True)
        out[src] = (len(paths), m.box.mp, m.box.mr, m.box.map50, m.box.map)
    return out


def video_sheets(onnx: str, video: str, n: int, roi_frac, conf: float, out_dir: str, tag: str) -> list:
    cap = cv2.VideoCapture(video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W, H = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    roi = None
    if roi_frac:
        x1, y1, x2, y2 = roi_frac
        roi = (int(x1 * W), int(y1 * H), int(x2 * W), int(y2 * H))
    det = KartDetector(onnx, conf=conf, roi=roi, providers=["CPUExecutionProvider"], warmup=1)
    os.makedirs(out_dir, exist_ok=True)
    items, counts = [], []
    for k in np.linspace(total * 0.02, total * 0.98, n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(k))
        ok, im = cap.read()
        if not ok:
            continue
        d = det(im)
        counts.append(len(d))
        if roi:
            cv2.rectangle(im, roi[:2], roi[2:], (255, 255, 0), 1)
        p = os.path.join(out_dir, f"{tag}_f{k}.jpg")
        cv2.imwrite(p, im)
        bx = [{"xyxy": b.tolist(), "id": i, "src": "accepted", "conf": float(s)} for i, (b, s) in enumerate(zip(d.boxes, d.scores))]
        items.append((p, bx, f"{tag} f{k} n={len(d)}"))
    sheets = render_sheets(items, out_dir, cols=4, rows=4, prefix=f"sheet_{tag}")
    for p, _, _ in items:
        os.remove(p)
    return sheets, counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", nargs="+", required=True, help="training run dirs (with weights/best.pt + kart_*.onnx)")
    ap.add_argument("--dataset", required=True, help="build_dataset.py output (with manifest.csv)")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--video", nargs="*", default=[])
    ap.add_argument("--frames", type=int, default=16)
    ap.add_argument("--roi-frac", type=float, nargs=4, default=None, metavar=("X1", "Y1", "X2", "Y2"),
                    help="crop as fractions of the frame, e.g. 0 0 1 0.6 drops the bottom 40%% (our hood)")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    print(f"{'run':22s} {'val subset':34s} {'n':>4s} {'P':>6s} {'R':>6s} {'mAP50':>6s} {'mAP50-95':>8s}")
    for run in args.runs:
        for src, (n, p, r, m50, m) in val_by_source(run, args.dataset, args.imgsz, args.device).items():
            print(f"{os.path.basename(run):22s} {src[:34]:34s} {n:4d} {p:6.3f} {r:6.3f} {m50:6.3f} {m:8.3f}")
    for run in args.runs:
        for onnx in sorted(glob.glob(os.path.join(run, "weights", "kart_*.onnx"))):
            for v in args.video:
                tag = f"{os.path.basename(run)}_{os.path.basename(onnx)[5:-5]}_{os.path.basename(v)[:8]}"
                sheets, counts = video_sheets(onnx, v, args.frames, args.roi_frac, args.conf, args.out, tag)
                print(f"{tag}: detections/frame {np.mean(counts):.2f}, frames with any {np.mean(np.array(counts) > 0):.0%} "
                      f"-> {sheets[0] if sheets else '-'}")


if __name__ == "__main__":
    main()
