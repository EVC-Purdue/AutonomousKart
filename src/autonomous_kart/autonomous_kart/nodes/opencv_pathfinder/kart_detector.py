"""
Kart detector: YOLO-family ONNX model behind ONNX Runtime.

One code path for every box we might run on; only the execution provider
changes:
  Jetson : TensorrtExecutionProvider (FP16/INT8 engine, cached on disk)
           -> CUDAExecutionProvider -> CPU
  Rubik  : QNNExecutionProvider (Hexagon NPU) -> CPU
  laptop : CoreML / CPU, for development and benchmarking

Accepts both Ultralytics ONNX output layouts:
  raw        (1, 4 + nc, N)  cx, cy, w, h + per-class scores; NMS done here
  end-to-end (1, K, 6)       x1, y1, x2, y2, score, class (YOLO26 / nms=True)

Frame -> boxes is three stages, each timed in `last_timing_ms`:
  pre   optional ROI crop, letterbox into a preallocated buffer, HWC->CHW
  infer session.run
  post  decode, threshold, NMS, map back to source-image pixels

Pre/post are plain numpy + cv2 and importable without onnxruntime, so the
unit tests (and CI) exercise them with synthetic tensors.
"""

import time
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

_PAD = 114  # Ultralytics letterbox fill
_INV255 = np.float32(1.0 / 255.0)  # float32 scalar keeps the multiply in float32


@dataclass
class Detections:
    boxes: np.ndarray  # (M, 4) float32 x1, y1, x2, y2 in source-image pixels
    scores: np.ndarray  # (M,) float32
    classes: np.ndarray  # (M,) int32

    def __len__(self) -> int:
        return int(self.scores.shape[0])


@dataclass(frozen=True)
class Letterbox:
    """Where the (cropped) source landed inside the network input."""

    scale: float
    pad_x: float
    pad_y: float
    roi_x: int
    roi_y: int


def _empty() -> Detections:
    return Detections(np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32))


class Preprocessor:
    """Letterbox into a reused buffer; returns a (1, 3, H, W) float32 blob.

    roi = (x1, y1, x2, y2) crops the source first: the sky and our own hood
    never contain a kart, and cropping at native resolution keeps far karts
    larger than a full-frame downscale would.
    """

    def __init__(self, input_hw: Tuple[int, int], roi: Optional[Tuple[int, int, int, int]] = None):
        self.h, self.w = int(input_hw[0]), int(input_hw[1])
        self.roi = tuple(int(v) for v in roi) if roi else None
        self._canvas = np.full((self.h, self.w, 3), _PAD, np.uint8)
        self._blob = np.empty((1, 3, self.h, self.w), np.float32)
        self._last_geom: Optional[Tuple[int, int, int, int]] = None

    def __call__(self, bgr: np.ndarray) -> Tuple[np.ndarray, Letterbox]:
        rx = ry = 0
        if self.roi is not None:
            x1, y1, x2, y2 = self.roi
            H, W = bgr.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W, x2), min(H, y2)
            # ROI outside this frame (camera came up at another resolution):
            # detect on the whole frame rather than on nothing.
            if x2 > x1 and y2 > y1:
                bgr = bgr[y1:y2, x1:x2]
                rx, ry = x1, y1
        sh, sw = bgr.shape[:2]
        scale = min(self.w / sw, self.h / sh)
        nw, nh = int(round(sw * scale)), int(round(sh * scale))
        px, py = (self.w - nw) // 2, (self.h - nh) // 2
        geom = (nw, nh, px, py)
        if geom != self._last_geom:  # only re-fill padding when the layout changes
            self._canvas[:] = _PAD
            self._last_geom = geom
        # INTER_LINEAR matches the Ultralytics predictor (and is the cheapest)
        cv2.resize(bgr, (nw, nh), dst=self._canvas[py:py + nh, px:px + nw], interpolation=cv2.INTER_LINEAR)
        # BGR->RGB, HWC->CHW, /255 in one pass into the reused blob
        np.multiply(self._canvas[:, :, ::-1].transpose(2, 0, 1), _INV255, out=self._blob[0])
        return self._blob, Letterbox(scale, float(px), float(py), rx, ry)


def _to_source(xyxy: np.ndarray, lb: Letterbox, src_hw: Tuple[int, int]) -> np.ndarray:
    out = np.empty_like(xyxy, dtype=np.float32)
    out[:, [0, 2]] = (xyxy[:, [0, 2]] - lb.pad_x) / lb.scale + lb.roi_x
    out[:, [1, 3]] = (xyxy[:, [1, 3]] - lb.pad_y) / lb.scale + lb.roi_y
    # Assign, don't clip(out=out[:, [0, 2]]): fancy indexing is a copy
    out[:, [0, 2]] = np.clip(out[:, [0, 2]], 0, src_hw[1] - 1)
    out[:, [1, 3]] = np.clip(out[:, [1, 3]], 0, src_hw[0] - 1)
    return out


def _nms(xyxy: np.ndarray, scores: np.ndarray, iou: float, max_det: int) -> np.ndarray:
    """Class-agnostic: two boxes on one kart are one kart whatever their labels."""
    xywh = np.column_stack([xyxy[:, 0], xyxy[:, 1], xyxy[:, 2] - xyxy[:, 0], xyxy[:, 3] - xyxy[:, 1]])
    keep = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 0.0, iou, top_k=max_det)
    return np.asarray(keep, dtype=np.int64).reshape(-1)


def postprocess(
    out: np.ndarray,
    lb: Letterbox,
    src_hw: Tuple[int, int],
    conf: float = 0.35,
    iou: float = 0.5,
    class_ids: Optional[Sequence[int]] = None,
    max_det: int = 20,
) -> Detections:
    """Decode either output layout into source-pixel detections."""
    out = np.asarray(out)
    if out.ndim == 3:
        out = out[0]
    if out.ndim != 2:
        raise ValueError(f"unexpected detector output shape {out.shape}")

    if out.shape[1] == 6 and out.shape[0] != 6:
        # End-to-end: rows are x1, y1, x2, y2, score, class (already NMS'd)
        scores = out[:, 4]
        cls = out[:, 5].astype(np.int32)
        m = scores >= conf
        if class_ids is not None:
            m &= np.isin(cls, class_ids)
        if not m.any():
            return _empty()
        idx = np.flatnonzero(m)
        idx = idx[np.argsort(-scores[idx])][:max_det]
        return Detections(_to_source(out[idx, :4], lb, src_hw), scores[idx].astype(np.float32), cls[idx])

    # Raw: (4 + nc, N). Threshold before anything else: N is thousands of
    # anchors and nearly all of them are background.
    cls_scores = out[4:]
    if class_ids is not None:
        cls_scores = cls_scores[list(class_ids)]
        cls_map = np.asarray(class_ids, dtype=np.int32)
    else:
        cls_map = np.arange(cls_scores.shape[0], dtype=np.int32)
    best = cls_scores.argmax(axis=0)
    scores = cls_scores[best, np.arange(cls_scores.shape[1])]
    idx = np.flatnonzero(scores >= conf)
    if idx.size == 0:
        return _empty()
    cx, cy, w, h = out[0, idx], out[1, idx], out[2, idx], out[3, idx]
    xyxy = np.column_stack([cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h])
    s = scores[idx]
    keep = _nms(xyxy, s, iou, max_det)
    return Detections(_to_source(xyxy[keep], lb, src_hw), s[keep].astype(np.float32), cls_map[best[idx[keep]]])


def default_providers(available: Sequence[str], engine_cache_dir: str = "/root/.cache/trt_engines",
                      fp16: bool = True) -> list:
    """Best-first provider list restricted to what this onnxruntime build has."""
    prefs = [
        ("TensorrtExecutionProvider", {
            "trt_fp16_enable": fp16,
            "trt_engine_cache_enable": True,  # engine build takes minutes; do it once
            "trt_engine_cache_path": engine_cache_dir,
        }),
        ("CUDAExecutionProvider", {}),
        ("QNNExecutionProvider", {"backend_path": "libQnnHtp.so", "htp_performance_mode": "burst"}),
        ("CoreMLExecutionProvider", {}),
        ("CPUExecutionProvider", {}),
    ]
    return [(n, o) for n, o in prefs if n in available]


class KartDetector:
    def __init__(
        self,
        model_path: str,
        conf: float = 0.35,
        iou: float = 0.5,
        class_ids: Optional[Sequence[int]] = None,
        roi: Optional[Tuple[int, int, int, int]] = None,
        providers: Optional[Sequence[str]] = None,
        threads: int = 2,
        fp16: bool = True,
        engine_cache_dir: str = "/root/.cache/trt_engines",
        warmup: int = 3,
    ):
        import onnxruntime as ort  # deferred so pre/post stay importable without it

        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # The MPC also wants CPU every 16 ms; don't let a CPU-fallback
        # detector grab every core.
        so.intra_op_num_threads = int(threads)
        so.inter_op_num_threads = 1
        avail = ort.get_available_providers()
        chosen = default_providers(avail, engine_cache_dir, fp16)
        if providers:
            wanted = list(providers)
            chosen = [p for p in chosen if p[0] in wanted]
            if not chosen:
                raise RuntimeError(f"none of {wanted} available (have {avail})")
        self.session = ort.InferenceSession(model_path, sess_options=so, providers=chosen)
        self.providers = self.session.get_providers()

        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        h, w = inp.shape[2], inp.shape[3]
        if not (isinstance(h, int) and isinstance(w, int)):
            raise ValueError("export the model with a fixed imgsz (dynamic=False)")
        self.input_hw = (h, w)
        self.pre = Preprocessor(self.input_hw, roi)
        self.conf, self.iou = float(conf), float(iou)
        self.class_ids = list(class_ids) if class_ids is not None else None
        self.last_timing_ms: Dict[str, float] = {}

        # Warm up on a frame the ROI actually fits in, so the warmup exercises the real path
        wh = max(h, roi[3]) if roi else h
        ww = max(w, roi[2]) if roi else w
        dummy = np.full((wh, ww, 3), _PAD, np.uint8)
        for _ in range(max(0, warmup)):  # first TRT/QNN run builds or loads the engine
            self(dummy)

    def __call__(self, bgr: np.ndarray) -> Detections:
        t0 = time.perf_counter()
        blob, lb = self.pre(bgr)
        t1 = time.perf_counter()
        out = self.session.run(None, {self.input_name: blob})[0]
        t2 = time.perf_counter()
        det = postprocess(out, lb, bgr.shape[:2], self.conf, self.iou, self.class_ids)
        t3 = time.perf_counter()
        self.last_timing_ms = {
            "pre": 1e3 * (t1 - t0), "infer": 1e3 * (t2 - t1), "post": 1e3 * (t3 - t2), "total": 1e3 * (t3 - t0),
        }
        return det
