#!/usr/bin/env python3
"""
Latency benchmark for the kart detector on whatever box this runs on.

Feeds real frames (an image folder or a video) through KartDetector exactly
as the node will, and reports per-stage latency percentiles. Run it on the
Jetson and on the Rubik to decide where detection lives.

    python3 scripts/bench_kart_detector.py --model kart_192x320.onnx \
        --source ../GoKartSampleSet/images --src-size 1280x720 --threads 2

    # Compare providers / sizes in one go
    python3 scripts/bench_kart_detector.py --model a.onnx b.onnx \
        --providers TensorrtExecutionProvider CUDAExecutionProvider CPUExecutionProvider

--src-size resizes inputs to the camera's resolution first (phone photos in
the sample set are 12 MP; the kart camera is not), so pre-processing cost
is representative.
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "autonomous_kart"))
from autonomous_kart.nodes.opencv_pathfinder.kart_detector import KartDetector  # noqa: E402


def load_frames(source: str, n: int, size):
    frames = []
    if os.path.isdir(source):
        paths = sorted(glob.glob(os.path.join(source, "*.jpg")) + glob.glob(os.path.join(source, "*.png")))
        for p in paths[:n]:
            im = cv2.imread(p)
            if im is not None:
                frames.append(im)
    else:
        cap = cv2.VideoCapture(source)
        while len(frames) < n:
            ok, im = cap.read()
            if not ok:
                break
            frames.append(im)
        cap.release()
    if not frames:
        raise SystemExit(f"no frames loaded from {source}")
    if size:
        frames = [cv2.resize(f, size, interpolation=cv2.INTER_AREA) for f in frames]
    return frames


def pct(a, q):
    return float(np.percentile(a, q))


def bench(model, frames, args, providers):
    det = KartDetector(
        model, conf=args.conf, roi=args.roi, providers=providers, threads=args.threads,
        fp16=not args.fp32, engine_cache_dir=args.engine_cache, warmup=args.warmup,
    )
    stages = {"pre": [], "infer": [], "post": [], "total": []}
    n_det = 0
    t_wall = time.perf_counter()
    for i in range(args.iters):
        d = det(frames[i % len(frames)])
        n_det += len(d)
        for k in stages:
            stages[k].append(det.last_timing_ms[k])
    wall = time.perf_counter() - t_wall
    name = os.path.basename(model)
    print(f"\n{name}  input {det.input_hw[0]}x{det.input_hw[1]}  providers {det.providers[0]}  "
          f"threads {args.threads}  frames {frames[0].shape[1]}x{frames[0].shape[0]}")
    print(f"  {'stage':6s} {'p50':>7s} {'p90':>7s} {'p99':>7s} {'max':>7s}  ms")
    for k, v in stages.items():
        v = np.asarray(v)
        print(f"  {k:6s} {pct(v, 50):7.2f} {pct(v, 90):7.2f} {pct(v, 99):7.2f} {v.max():7.2f}")
    print(f"  throughput {args.iters / wall:.0f} fps, {n_det / args.iters:.2f} dets/frame, "
          f"p99 total {'fits' if pct(stages['total'], 99) < args.budget_ms else 'EXCEEDS'} "
          f"{args.budget_ms:.1f} ms budget")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", nargs="+", required=True, help="ONNX model(s) exported with fixed imgsz")
    ap.add_argument("--source", required=True, help="image folder or video file")
    ap.add_argument("--src-size", default="1280x720", help="resize frames to WxH first ('' to keep)")
    ap.add_argument("--providers", nargs="*", default=None,
                    help="restrict to these execution providers (each benchmarked separately)")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--iters", type=int, default=500)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--roi", type=int, nargs=4, default=None, metavar=("X1", "Y1", "X2", "Y2"))
    ap.add_argument("--fp32", action="store_true", help="disable TensorRT FP16")
    ap.add_argument("--engine-cache", default=os.path.expanduser("~/.cache/trt_engines"))
    ap.add_argument("--budget-ms", type=float, default=1000.0 / 60.0)
    args = ap.parse_args()

    size = tuple(int(v) for v in args.src_size.lower().split("x")) if args.src_size else None
    frames = load_frames(args.source, 200, size)
    provider_sets = [[p] for p in args.providers] if args.providers else [None]
    for model in args.model:
        for prov in provider_sets:
            try:
                bench(model, frames, args, prov)
            except Exception as e:  # keep going so one missing provider doesn't kill the sweep
                print(f"\n{os.path.basename(model)} {prov}: FAILED {e}")


if __name__ == "__main__":
    main()
