# perception: kart detector dataset + training

This folder holds the offline tooling that produces the ONNX model
`nodes/opencv_pathfinder/kart_detector.py` runs. It is not part of the ROS package.
Keep data, runs and weights **outside the repo**. The examples use `../datasets` and
`../runs`, both next to `AutonomousKart/`.

Env: the `kart` conda env (ultralytics 8.4, onnxruntime, opencv). Run the
commands from `../datasets/_weights` so the big YOLOE and MobileCLIP weights
download there once.

```
R=$(pwd)/../../AutonomousKart     # from ../datasets/_weights
```

## 1. Auto-label

```
python $R/perception/autolabel.py compare --images IMG_DIR --out ../autolabel_compare \
    --prompts "go-kart" "go kart" "kart" "racing kart"     # pick prompts by eye
python $R/perception/autolabel.py label --images IMG_DIR --out ../autolabel_mine \
    --prompts "go-kart" "racing kart"
```

The labeler is YOLOE-11L, prompted with text, at imgsz 1280. We tried YOLOE-26L
and it boxed only part of the kart (just the wheels and seat, for example).
`label` writes `candidates.json`, which holds every box down to conf 0.05, plus
`labels/` (boxes at or above `--conf`, default 0.25) and contact sheets in `sheets/`.

## 2. Review

Look through `sheets/*.jpg`. Green boxes were accepted. Thin orange boxes are
rejected candidates, and their ids are drawn on the sheet. Record fixes in
`OUT/review.yaml` (the format is in the `autolabel.py` docstring), then rebuild:

```
python $R/perception/autolabel.py apply --out ../autolabel_mine
```

What a box covers: the kart body, including wheels, bumpers, steering wheel
and light bar. It excludes the thin GPS mast, any dolly or stand the kart is
parked on, and people. Common YOLOE mistakes on our kart:
- extending the box up the GPS mast
- splitting the kart into two boxes
- boxing the kart together with its dolly
- missing a far wheel

Check every val image by hand.

## 3. Build the dataset

**Roboflow export:** export as "YOLOv8" or "YOLO11" format and unzip it to
`../datasets/roboflow_<name>/`. That folder should hold `data.yaml` plus
`train/`, `valid/` and `test/`, each with `images/` and `labels/`. List
hand-labeled sources first, because when two images are near-duplicates the
first source wins:

```
python $R/perception/build_dataset.py --out ../kart_v1 \
    --source ../roboflow_<name> ../autolabel_gokart \
    --val-list ../val_list_v0.txt        # keeps the hand-verified val set fixed
```

How classes are handled:
- Classes named kart, go-kart, gokart, racing kart (and similar) map to class 0.
- All other classes are dropped.
- Add synonyms with `--kart-names car vehicle` (check the printed class map).
- Polygons are reduced to their bounding box.

Other processing:
- Images are resized so the long side is at most 1280.
- Near-duplicates (pHash) are removed.
- The split keeps each group whole. A group is one capture sequence (nearby
  file numbers) or a set of perceptually similar shots. A group goes entirely
  to train or entirely to val, so val is never a copy of train.

Without `--val-list`, about 18% of the groups go to val. With it, *only* the
listed images (and the rest of their groups) go to val. Look at
`OUT/val_sheets/` after every build.

## 4. Train, export, benchmark

```
python $R/perception/train.py --data ../kart_v1/data.yaml --model yolo11n.pt yolo26n.pt \
    --project ../../runs --name v1 --bench-source ../../GoKartSampleSet/images
```

For each model this trains at imgsz 320 with early stopping, then validates.
It exports `runs/<name>_<model>/weights/kart_192x320.onnx` and
`kart_256x448.onnx` (opset 17, simplified, static shape) and prints the
`scripts/bench_kart_detector.py` command for CPU and CoreML.

Augmentations:
- horizontal flip, but never vertical
- HSV and brightness jitter
- scale and mosaic
- a custom linear motion blur (see `train.py`)
