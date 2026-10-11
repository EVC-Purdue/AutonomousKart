#!/usr/bin/env python3
"""
Open-vocabulary auto-labeling of an image folder into YOLO labels (class 0 = kart).

A big open-vocab detector (YOLOE, text-prompted) proposes boxes; a human then
reviews contact sheets and records fixes in a review file, which `apply` turns
into the final labels. Every step is re-runnable, so fixes are never lost when
re-labeling.

    # 1. compare prompts on a few images (one panel per prompt)
    python perception/autolabel.py compare --images IMG_DIR --out OUT --prompts "go-kart" "go kart" "kart"
    # 2. label: writes OUT/candidates.json (all boxes >= --min-conf) + OUT/labels + OUT/sheets
    python perception/autolabel.py label --images IMG_DIR --out OUT --prompts "go-kart"
    # 3. review OUT/sheets/*.jpg, edit OUT/review.yaml, then rebuild labels + sheets
    python perception/autolabel.py apply --out OUT

review.yaml (keys are image stems, '.rf.<hash>' optional; ids are the numbers drawn on the sheets):
    some_image_stem:
      keep: [0, 3]               # exactly these candidate ids (default: all >= --conf)
      drop: [1]                  # or: default set minus these
      add: [[x1, y1, x2, y2]]    # manual boxes, source-image pixels
      fix: {0: [x1, y1, x2, y2]} # replace a candidate's box (e.g. it swallowed a person)
      add_pct / fix_pct          # same, but x1, y1, x2, y2 in percent of width/height
    another_stem: {empty: true}  # verified: no karts
    third_stem: {ok: true}       # verified as-is

Sheets: green = accepted (bold id), orange = rejected candidate (thin), blue = manual.
"""

import argparse
import glob
import json
import math
import os
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
import yaml

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
TILE = (400, 300)  # contact-sheet cell, w x h


def list_images(folder: str) -> List[str]:
    return sorted(p for p in glob.glob(os.path.join(folder, "*")) if p.lower().endswith(IMG_EXT))


def load_model(weights: str, prompts: Sequence[str]):
    from ultralytics import YOLOE

    model = YOLOE(weights)
    model.set_classes(list(prompts), model.get_text_pe(list(prompts)))
    return model


def detect(model, paths: Sequence[str], min_conf: float, imgsz: int, device: str) -> Dict[str, list]:
    """stem -> [[x1, y1, x2, y2, conf], ...] in source pixels, best first."""
    out = {}
    for p in paths:
        # agnostic NMS: every prompt is a synonym for the one class
        r = model.predict(p, conf=min_conf, iou=0.5, agnostic_nms=True, imgsz=imgsz, device=device,
                          verbose=False, max_det=50)[0]
        b = r.boxes
        rows = np.column_stack([b.xyxy.cpu().numpy(), b.conf.cpu().numpy()]) if len(b) else np.zeros((0, 5))
        rows = rows[np.argsort(-rows[:, 4])]
        out[stem(p)] = {"path": os.path.abspath(p), "hw": list(r.orig_shape),
                        "boxes": [[round(float(v), 1) for v in row[:4]] + [round(float(row[4]), 3)] for row in rows]}
    return out


def stem(p: str) -> str:
    return os.path.splitext(os.path.basename(p))[0]


def resolve(cand: dict, rev: Optional[dict], conf: float) -> List[dict]:
    """Final boxes for one image: [{'xyxy', 'id', 'src'}], src in accepted/manual/rejected."""
    boxes = cand["boxes"]
    rev = rev or {}
    if rev.get("empty"):
        keep = set()
    elif "keep" in rev:
        keep = set(rev["keep"])
    else:
        keep = {i for i, b in enumerate(boxes) if b[4] >= conf} - set(rev.get("drop", []))
    w, h = cand["hw"][1], cand["hw"][0]
    px = lambda b: [b[0] * w / 100, b[1] * h / 100, b[2] * w / 100, b[3] * h / 100]  # noqa: E731  % -> pixels
    fix = {int(k): v for k, v in (rev.get("fix") or {}).items()}
    fix.update({int(k): px(v) for k, v in (rev.get("fix_pct") or {}).items()})
    keep |= set(fix)
    out = []
    for i, b in enumerate(boxes):
        out.append({"xyxy": fix.get(i, b[:4]), "id": i, "src": "accepted" if i in keep else "rejected", "conf": b[4]})
    if not rev.get("empty"):
        for j, b in enumerate((rev.get("add") or []) + [px(b) for b in rev.get("add_pct") or []]):
            out.append({"xyxy": b, "id": f"m{j}", "src": "manual", "conf": 1.0})
    return out


def write_labels(cands: Dict[str, dict], review: dict, out: str, conf: float) -> Dict[str, int]:
    lbl_dir, img_dir = os.path.join(out, "labels"), os.path.join(out, "images")
    os.makedirs(lbl_dir, exist_ok=True)
    os.makedirs(img_dir, exist_ok=True)
    stats = {"images": 0, "boxes": 0, "empty": 0, "reviewed": 0}
    for s, c in cands.items():
        h, w = c["hw"]
        lines = []
        for b in resolve(c, review.get(s), conf):
            if b["src"] == "rejected":
                continue
            x1, y1, x2, y2 = np.clip(b["xyxy"], 0, [w, h, w, h])
            lines.append(f"0 {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
        with open(os.path.join(lbl_dir, s + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        # symlink, not copy: build_dataset.py re-encodes (and downsizes) anyway
        link = os.path.join(img_dir, os.path.basename(c["path"]))
        if not os.path.lexists(link):
            os.symlink(c["path"], link)
        stats["images"] += 1
        stats["boxes"] += len(lines)
        stats["empty"] += not lines
        stats["reviewed"] += s in review
    return stats


def tile(img: np.ndarray, boxes: List[dict], title: str) -> np.ndarray:
    h, w = img.shape[:2]
    s = min(TILE[0] / w, (TILE[1] - 18) / h)
    im = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    cell = np.full((TILE[1], TILE[0], 3), 40, np.uint8)
    cell[18:18 + im.shape[0], :im.shape[1]] = im
    colors = {"accepted": (0, 230, 0), "rejected": (0, 140, 255), "manual": (255, 120, 0)}
    for b in sorted(boxes, key=lambda b: b["src"] != "rejected"):  # rejected underneath
        x1, y1, x2, y2 = (int(v * s) for v in b["xyxy"])
        y1, y2 = y1 + 18, y2 + 18
        thick = 1 if b["src"] == "rejected" else 2
        cv2.rectangle(cell, (x1, y1), (x2, y2), colors[b["src"]], thick)
        lab = f"{b['id']}" if b["src"] == "manual" else f"{b['id']}:{b['conf']:.2f}"
        cv2.putText(cell, lab, (x1 + 2, max(y1 + 12, 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[b["src"]], 1, cv2.LINE_AA)
    cv2.putText(cell, title[:60], (3, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return cell


def render_sheets(items: List[tuple], out_dir: str, cols: int = 4, rows: int = 4, prefix: str = "sheet") -> List[str]:
    """items: (image_path, boxes, title). Writes cols x rows grids; returns the paths."""
    os.makedirs(out_dir, exist_ok=True)
    for old in glob.glob(os.path.join(out_dir, f"{prefix}_*.jpg")):
        os.remove(old)
    per, paths = cols * rows, []
    for k in range(math.ceil(len(items) / per)):
        cells = [tile(cv2.imread(p), b, t) for p, b, t in items[k * per:(k + 1) * per]]
        cells += [np.full((TILE[1], TILE[0], 3), 40, np.uint8)] * (per - len(cells))
        grid = np.vstack([np.hstack(cells[r * cols:(r + 1) * cols]) for r in range(rows)])
        paths.append(os.path.join(out_dir, f"{prefix}_{k:03d}.jpg"))
        cv2.imwrite(paths[-1], grid, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return paths


def sheets_from_candidates(cands: Dict[str, dict], review: dict, out: str, conf: float) -> List[str]:
    items = []
    for i, (s, c) in enumerate(cands.items()):
        mark = "*" if s in review else ""
        items.append((c["path"], resolve(c, review.get(s), conf), f"{i}{mark} {s.split('.rf.')[0]}"))
    return render_sheets(items, os.path.join(out, "sheets"))


def load_review(out: str) -> dict:
    p = os.path.join(out, "review.yaml")
    if not os.path.exists(p):
        return {}
    with open(p) as f:
        return yaml.safe_load(f) or {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("compare", "label"):
        p = sub.add_parser(name)
        p.add_argument("--images", required=True)
        p.add_argument("--out", required=True)
        p.add_argument("--prompts", nargs="+", default=["go-kart"])
        p.add_argument("--weights", default="yoloe-11l-seg.pt", help="any Ultralytics YOLOE checkpoint")
        p.add_argument("--imgsz", type=int, default=1280, help="large: parked karts can be small and far")
        p.add_argument("--min-conf", type=float, default=0.05, help="keep candidates down to this for review")
        p.add_argument("--conf", type=float, default=0.25, help="auto-accept threshold")
        p.add_argument("--device", default="mps")
    sub.choices["compare"].add_argument("-n", type=int, default=8, help="images to compare on (evenly spaced)")
    p = sub.add_parser("apply", help="re-apply review.yaml to candidates.json, rewrite labels + sheets")
    p.add_argument("--out", required=True)
    p.add_argument("--conf", type=float, default=None, help="default: the value used at label time")
    args = ap.parse_args()

    if args.cmd == "compare":
        paths = list_images(args.images)
        paths = paths[:: max(1, len(paths) // args.n)][:args.n]
        items = []
        for prompt in args.prompts:
            c = detect(load_model(args.weights, [prompt]), paths, args.min_conf, args.imgsz, args.device)
            n = sum(sum(b[4] >= args.conf for b in v["boxes"]) for v in c.values())
            print(f"{prompt!r}: {n} boxes >= {args.conf} on {len(paths)} images")
            items += [(v["path"], resolve(v, None, args.conf), f"{prompt} | {s.split('.rf.')[0]}") for s, v in c.items()]
        # one row per prompt, same images in each column
        for p in render_sheets(items, os.path.join(args.out, "compare"), cols=len(paths), rows=len(args.prompts),
                               prefix="compare"):
            print("wrote", p)
        return

    if args.cmd == "label":
        os.makedirs(args.out, exist_ok=True)
        model = load_model(args.weights, args.prompts)
        cands = detect(model, list_images(args.images), args.min_conf, args.imgsz, args.device)
        meta = {"weights": args.weights, "prompts": args.prompts, "imgsz": args.imgsz, "conf": args.conf}
        with open(os.path.join(args.out, "candidates.json"), "w") as f:
            json.dump({"meta": meta, "images": cands}, f, indent=1)
        conf = args.conf
    else:
        with open(os.path.join(args.out, "candidates.json")) as f:
            d = json.load(f)
        cands, conf = d["images"], args.conf if args.conf is not None else d["meta"]["conf"]

    # Keys may drop Roboflow's '.rf.<hash>' suffix ('IMG_9081_JPG' for 'IMG_9081_JPG.rf.ab12...')
    short = {s.split(".rf.")[0]: s for s in cands}
    review = {short.get(k, k): v for k, v in load_review(args.out).items()}
    unknown = set(review) - set(cands)
    if unknown:
        raise SystemExit(f"review.yaml has stems not in candidates.json: {sorted(unknown)[:5]}")
    stats = write_labels(cands, review, args.out, conf)
    sheets = sheets_from_candidates(cands, review, args.out, conf)
    print(f"{stats} -> {args.out}/labels, {len(sheets)} sheets in {args.out}/sheets")


if __name__ == "__main__":
    main()
