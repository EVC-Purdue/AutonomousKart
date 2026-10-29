#!/usr/bin/env python3
"""
Merge YOLO-format sources into one single-class (kart) Ultralytics dataset.

    python perception/build_dataset.py --out DATASETS/kart_v0 \
        --source DATASETS/roboflow_export DATASETS/autolabel_gokart --val-frac 0.18

A source is either
  * a Roboflow "YOLOv8/YOLO11" export: data.yaml + {train,valid,test}/{images,labels}
    (all splits are pooled; we re-split ourselves), or
  * a flat folder with images/ and labels/ (autolabel.py output). With no
    data.yaml its class 0 is taken to be kart.
Class names meaning kart (see KART_NAMES) map to class 0; every other class is
dropped. Segmentation polygons are reduced to their bounding box.

Then:
  * images are re-encoded with the long side <= --max-side (12 MP phone shots
    only slow the dataloader; the model trains at 320 anyway),
  * with --restore-aspect, images a Roboflow export stretched to a square are
    resized back to their original aspect (from a 'WxH' in the filename, or
    from a same-named image in another source). YOLO labels are normalized,
    so they stay valid; the camera never sees squashed karts,
  * an image whose original name already came from an earlier source is
    dropped (stretching defeats the perceptual hash, so name matching catches
    exports of images we already have),
  * near-duplicates (perceptual hash distance <= --dup-dist) are dropped; the
    source listed FIRST wins, so list hand-labeled exports before autolabels,
  * images are grouped (same capture sequence or perceptually similar) and
    whole groups go to train or val, so near-identical frames never straddle
    the split and inflate val mAP.
Writes OUT/{images,labels}/{train,val}, OUT/data.yaml, OUT/manifest.csv and
val contact sheets in OUT/val_sheets for a human check.
"""

import argparse
import csv
import os
import random
import re
import shutil
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from autolabel import IMG_EXT, render_sheets  # noqa: E402

KART_NAMES = {"kart", "karts", "go-kart", "go-karts", "go kart", "go karts", "gokart", "gokarts", "go_kart",
              "racing kart", "race kart"}  # anything else (e.g. 'car'): opt in with --kart-names


def norm_name(n: str) -> str:
    return re.sub(r"\s+", " ", str(n).strip().lower())


def source_items(src: str, kart_names: set) -> List[dict]:
    """Every (image, boxes) pair in a source, boxes as normalized cx, cy, w, h."""
    yml = os.path.join(src, "data.yaml")
    if os.path.exists(yml):
        with open(yml) as f:
            names = yaml.safe_load(f).get("names", {})
        names = dict(enumerate(names)) if isinstance(names, list) else {int(k): v for k, v in names.items()}
    else:
        names = {0: "kart"}
    keep = {k for k, v in names.items() if norm_name(v) in kart_names}
    print(f"{src}: classes {names} -> kart ids {sorted(keep)}")
    if not keep:
        print("  WARNING: no class maps to kart; pass --kart-names", file=sys.stderr)

    img_dirs = [d for d in (os.path.join(src, s, "images") for s in ("train", "valid", "val", "test")) if os.path.isdir(d)]
    img_dirs = img_dirs or [os.path.join(src, "images")]
    items = []
    for d in img_dirs:
        lbl_dir = os.path.join(os.path.dirname(d), "labels")
        for fn in sorted(os.listdir(d)):
            if not fn.lower().endswith(IMG_EXT):
                continue
            lp = os.path.join(lbl_dir, os.path.splitext(fn)[0] + ".txt")
            if not os.path.exists(lp):
                continue  # unlabeled != empty; skip rather than teach "no kart here"
            boxes = []
            for line in open(lp).read().split("\n"):
                v = line.split()
                if len(v) < 5 or int(float(v[0])) not in keep:
                    continue
                c = np.asarray(v[1:], np.float64)
                if len(c) > 4:  # polygon -> bbox
                    xs, ys = c[0::2], c[1::2]
                    c = np.array([(xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2, np.ptp(xs), np.ptp(ys)])
                boxes.append(c[:4])
            items.append({"img": os.path.realpath(os.path.join(d, fn)), "boxes": boxes, "source": os.path.basename(os.path.abspath(src))})
    return items


def orig_name(path: str) -> str:
    """Roboflow appends '_jpg.rf.<hash>'; strip it so augmented copies group together."""
    s = os.path.splitext(os.path.basename(path))[0]
    return re.split(r"_(?:jpe?g|png|JPE?G|PNG)\.rf\.|\.rf\.", s)[0]


def rf_stem(path: str) -> str:
    """Stem with only the '.rf.<hash>' part removed ('IMG_9081_JPG'), as autolabel review keys use."""
    return os.path.splitext(os.path.basename(path))[0].split(".rf.")[0]


def orig_aspect(name: str, known: Dict[str, float]) -> Optional[float]:
    """Original w/h for a (possibly stretched) export: 'WxH' in the name, else a same-named image elsewhere."""
    m = re.search(r"(\d{3,5})x(\d{3,5})", name)
    if m:
        return int(m.group(1)) / int(m.group(2))
    return known.get(name)


def seq_key(name: str, block: int) -> str:
    """Bucket filenames that differ only by a nearby trailing number (burst / video frames)."""
    m = re.match(r"^(.*?)(\d+)\D*$", name)
    return f"{m.group(1)}#{int(m.group(2)) // block}" if m else name


def phash(img: np.ndarray) -> int:
    g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    d = cv2.dct(g)[:8, :8].flatten()
    bits = d > np.median(d[1:])
    return int("".join("1" if b else "0" for b in bits), 2)


def hamming(a: int, b: int) -> int:
    return bin(a ^ b).count("1")


class DSU:
    def __init__(self, n: int):
        self.p = list(range(n))

    def find(self, i: int) -> int:
        while self.p[i] != i:
            self.p[i] = self.p[self.p[i]]
            i = self.p[i]
        return i

    def union(self, a: int, b: int) -> None:
        self.p[self.find(a)] = self.find(b)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", nargs="+", required=True, help="YOLO sources, highest label quality first")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-side", type=int, default=1280)
    ap.add_argument("--dup-dist", type=int, default=4, help="phash bits; <= this is a duplicate")
    ap.add_argument("--group-dist", type=int, default=12, help="phash bits; <= this joins a split group")
    ap.add_argument("--seq-block", type=int, default=10, help="filename numbers within one block share a group")
    ap.add_argument("--val-frac", type=float, default=0.18)
    ap.add_argument("--val-list", default=None, help="file of image names (Roboflow '.rf.' suffix stripped) forced into val; "
                    "only these go to val, so the hand-verified val set stays fixed across rebuilds")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--kart-names", nargs="*", default=None, help="extra class names to treat as kart")
    ap.add_argument("--restore-aspect", action="store_true",
                    help="un-stretch square (Roboflow 'Stretch') exports to their original aspect")
    args = ap.parse_args()

    kart_names = KART_NAMES | {norm_name(n) for n in (args.kart_names or [])}
    items = [it for s in args.source for it in source_items(s, kart_names)]
    print(f"{len(items)} labeled images from {len(args.source)} source(s)")

    # Same original name from an earlier (higher-priority) source: drop.
    seen_src: Dict[str, str] = {}
    by_name = []
    for it in items:
        n = orig_name(it["img"])
        if seen_src.setdefault(n, it["source"]) != it["source"]:
            continue
        by_name.append(it)
    print(f"dropped {len(items) - len(by_name)} images already present in an earlier source (by name)")
    items = by_name

    # Hash on a small decode; 12 MP JPEG decode is the slow part, so do it once
    imgs: List[Optional[np.ndarray]] = []
    known_aspect: Dict[str, float] = {}
    restored = 0
    for it in items:
        im = cv2.imread(it["img"])
        if im is not None and args.restore_aspect:
            h, w = im.shape[:2]
            if abs(w - h) <= 1:  # square = stretched export
                a = orig_aspect(orig_name(it["img"]), known_aspect)
                if a and abs(a - 1.0) > 0.02:
                    nw, nh = (w, round(w / a)) if a > 1 else (round(h * a), h)
                    im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_AREA)
                    restored += 1
            else:
                known_aspect[orig_name(it["img"])] = w / h
        if im is None:
            print("  unreadable, skipped:", it["img"])
        elif max(im.shape[:2]) > args.max_side:
            s = args.max_side / max(im.shape[:2])
            im = cv2.resize(im, (round(im.shape[1] * s), round(im.shape[0] * s)), interpolation=cv2.INTER_AREA)
        imgs.append(im)
        it["hash"] = phash(im) if im is not None else None
    keep = [i for i, im in enumerate(imgs) if im is not None]
    if args.restore_aspect:
        print(f"restored aspect on {restored} stretched images")

    # Dedupe: first occurrence (= higher-priority source) wins
    kept: List[int] = []
    dups = 0
    for i in keep:
        if any(hamming(items[i]["hash"], items[j]["hash"]) <= args.dup_dist for j in kept):
            dups += 1
            continue
        kept.append(i)
    print(f"dropped {dups} near-duplicates, {len(kept)} remain")

    # Groups: same source + nearby sequence number, or perceptually similar across anything
    dsu = DSU(len(kept))
    first: Dict[str, int] = {}
    for k, i in enumerate(kept):
        key = items[i]["source"] + "/" + seq_key(orig_name(items[i]["img"]), args.seq_block)
        if key in first:
            dsu.union(k, first[key])
        else:
            first[key] = k
    for a in range(len(kept)):
        for b in range(a + 1, len(kept)):
            if hamming(items[kept[a]]["hash"], items[kept[b]]["hash"]) <= args.group_dist:
                dsu.union(a, b)
    groups: Dict[int, List[int]] = defaultdict(list)
    for k, i in enumerate(kept):
        groups[dsu.find(k)].append(i)
    groups_l = list(groups.values())

    forced = set()
    if args.val_list:
        forced = {l.strip() for l in open(args.val_list) if l.strip()}
    rng = random.Random(args.seed)
    rng.shuffle(groups_l)
    split: Dict[int, str] = {}
    n_val_target = round(args.val_frac * len(kept))
    n_val = 0
    for g in groups_l:  # forced groups first, then fill to the target fraction
        if any(forced & {orig_name(items[i]["img"]), rf_stem(items[i]["img"])} for i in g):
            for i in g:
                split[i] = "val"
            n_val += len(g)
    for g in groups_l:
        if g[0] in split:
            continue
        to_val = not forced and n_val + len(g) <= n_val_target * 1.1 and n_val < n_val_target
        for i in g:
            split[i] = "val" if to_val else "train"
        n_val += len(g) if to_val else 0

    if os.path.isdir(args.out):
        for sub in ("images", "labels", "val_sheets"):
            shutil.rmtree(os.path.join(args.out, sub), ignore_errors=True)
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)

    rows, val_items = [], []
    for k, i in enumerate(kept):
        it, im, sp = items[i], imgs[i], split[i]
        stem = os.path.splitext(os.path.basename(it["img"]))[0]
        name = f"{it['source']}__{stem}"
        ip = os.path.join(args.out, "images", sp, name + ".jpg")
        cv2.imwrite(ip, im, [cv2.IMWRITE_JPEG_QUALITY, 95])
        with open(os.path.join(args.out, "labels", sp, name + ".txt"), "w") as f:
            f.writelines(f"0 {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}\n" for b in it["boxes"])
        rows.append({"name": name, "split": sp, "group": dsu.find(k), "source": it["source"],
                     "orig": it["img"], "boxes": len(it["boxes"]), "w": im.shape[1], "h": im.shape[0]})
        if sp == "val":
            h, w = im.shape[:2]
            bx = [{"xyxy": [(b[0] - b[2] / 2) * w, (b[1] - b[3] / 2) * h, (b[0] + b[2] / 2) * w, (b[1] + b[3] / 2) * h],
                   "id": j, "src": "manual", "conf": 1.0} for j, b in enumerate(it["boxes"])]
            val_items.append((ip, bx, f"{len(val_items)} {stem.split('.rf.')[0]}"))

    with open(os.path.join(args.out, "manifest.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    data = {"path": os.path.abspath(args.out), "train": "images/train", "val": "images/val", "names": {0: "kart"}}
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    sheets = render_sheets(val_items, os.path.join(args.out, "val_sheets"), prefix="val")

    n_tr = sum(r["split"] == "train" for r in rows)
    n_box = {s: sum(r["boxes"] for r in rows if r["split"] == s) for s in ("train", "val")}
    print(f"train {n_tr} imgs / {n_box['train']} boxes, val {len(rows) - n_tr} imgs / {n_box['val']} boxes, "
          f"{len(groups_l)} groups -> {args.out}/data.yaml ({len(sheets)} val sheets)")


if __name__ == "__main__":
    main()
