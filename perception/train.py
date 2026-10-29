#!/usr/bin/env python3
"""
Train a single-class kart detector, evaluate it, and export fixed-shape ONNX for KartDetector.

    python perception/train.py --data DATASETS/kart_v0/data.yaml --model yolo11n.pt yolo26n.pt \
        --project RUNS --name v0

Per model: train at imgsz 320 (early stopping), validate best.pt, export
<run>/weights/kart_<H>x<W>.onnx for every --export-hw (opset 17, simplified,
static shape: TensorRT/QNN want static shapes and KartDetector refuses dynamic
ones), then print the bench_kart_detector.py command.

Augmentation targets onboard footage: horizontal flips (tracks turn both
ways) but never vertical, strong brightness/saturation jitter (sun, shade,
dusk), scale + mosaic for near/far karts, and a linear motion blur the stock
pipeline lacks (camera shake, panning, a kart passing at speed).
"""

import argparse
import os
import shutil
from copy import copy
from typing import List, Tuple

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.data.dataset import YOLODataset
from ultralytics.models.yolo.detect import DetectionTrainer

AUG = dict(
    fliplr=0.5, flipud=0.0,
    hsv_h=0.015, hsv_s=0.6, hsv_v=0.5,
    degrees=2.0, translate=0.1, scale=0.5, shear=0.0, perspective=0.0,
    mosaic=1.0, close_mosaic=15, mixup=0.0,
)
BLUR_P = 0.3  # fraction of training images given motion blur


class MotionBlur:
    """Random linear blur; mostly horizontal, since pans and passing karts move sideways in frame."""

    def __init__(self, p: float = BLUR_P, k: Tuple[int, int] = (3, 7)):
        self.p, self.k = p, k

    def __call__(self, labels: dict) -> dict:
        if np.random.rand() >= self.p:
            return labels
        k = int(np.random.randint(self.k[0], self.k[1] + 1)) | 1
        kern = np.zeros((k, k), np.float32)
        kern[k // 2, :] = 1.0
        ang = np.random.uniform(-30, 30) if np.random.rand() < 0.8 else np.random.uniform(0, 180)
        rot = cv2.getRotationMatrix2D(((k - 1) / 2, (k - 1) / 2), ang, 1.0)
        kern = cv2.warpAffine(kern, rot, (k, k))
        labels["img"] = cv2.filter2D(labels["img"], -1, kern / max(kern.sum(), 1e-6))
        return labels


class KartDataset(YOLODataset):
    # Override build_transforms (not .transforms) so the blur survives
    # close_mosaic, which rebuilds the pipeline for the last epochs.
    def build_transforms(self, hyp):
        t = super().build_transforms(hyp)
        if self.augment:
            t.insert(len(t.transforms) - 1, MotionBlur())  # before Format (tensor conversion)
        return t


class KartTrainer(DetectionTrainer):
    def build_dataset(self, img_path, mode="train", batch=None):
        ds = super().build_dataset(img_path, mode, batch)
        if mode == "train":
            # build_yolo_dataset hardcodes YOLODataset; swap the class rather than copy its arg list.
            # Module-level classes so macOS 'spawn' dataloader workers can unpickle them.
            ds.__class__ = KartDataset
            ds.transforms = ds.build_transforms(hyp=copy(self.args))  # base class copies too
        return ds


def export_all(weights: str, hws: List[Tuple[int, int]]) -> List[str]:
    out = []
    for h, w in hws:
        # Each export writes best.onnx; rename so sizes don't overwrite each other
        src = YOLO(weights).export(format="onnx", imgsz=[h, w], opset=17, simplify=True, dynamic=False,
                                   half=False, device="cpu")
        dst = os.path.join(os.path.dirname(weights), f"kart_{h}x{w}.onnx")
        shutil.move(src, dst)
        out.append(dst)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="data.yaml from build_dataset.py")
    ap.add_argument("--model", nargs="+", default=["yolo11n.pt"], help="pretrained checkpoint(s), e.g. yolo26n.pt")
    ap.add_argument("--project", required=True, help="runs directory (keep it outside the repo)")
    ap.add_argument("--name", default="v0")
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--patience", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--export-hw", nargs="+", default=["192x320", "256x448"], help="HxW ONNX input sizes")
    ap.add_argument("--bench-source", default="../GoKartSampleSet/images")
    args = ap.parse_args()

    hws = [tuple(int(v) for v in s.lower().split("x")) for s in args.export_hw]
    exported, summary = [], []
    for m in args.model:
        name = f"{args.name}_{os.path.splitext(os.path.basename(m))[0]}"
        model = YOLO(m)
        model.train(trainer=KartTrainer, data=args.data, epochs=args.epochs, patience=args.patience,
                    imgsz=args.imgsz, batch=args.batch, device=args.device, workers=args.workers,
                    project=os.path.abspath(args.project), name=name, exist_ok=True, single_cls=True,
                    cache="ram", plots=True, seed=0, **AUG)
        best = os.path.join(os.path.abspath(args.project), name, "weights", "best.pt")
        met = YOLO(best).val(data=args.data, imgsz=args.imgsz, device=args.device, batch=args.batch,
                             plots=False, project=os.path.abspath(args.project), name=name + "_val", exist_ok=True)
        p, r, m50, m5095 = met.box.mp, met.box.mr, met.box.map50, met.box.map
        summary.append(f"{name:24s} P {p:.3f}  R {r:.3f}  mAP50 {m50:.3f}  mAP50-95 {m5095:.3f}")
        exported += export_all(best, hws)

    print("\n== val ==\n" + "\n".join(summary))
    print("\n== ONNX ==\n" + "\n".join(exported))
    repo = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    print(f"\nBenchmark:\n  cd '{repo}' && python scripts/bench_kart_detector.py \\\n"
          f"      --model {' '.join(repr(e) for e in exported)} \\\n"
          f"      --source '{os.path.abspath(args.bench_source)}' --src-size 1280x720 --threads 2 \\\n"
          "      --providers CPUExecutionProvider CoreMLExecutionProvider")


if __name__ == "__main__":
    main()
