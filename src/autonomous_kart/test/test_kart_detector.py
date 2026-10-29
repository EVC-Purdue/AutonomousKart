"""Unit tests for the kart detector's pre/post-processing and runtime wrapper."""
import numpy as np
import pytest

from autonomous_kart.nodes.opencv_pathfinder.kart_detector import (
    Letterbox, Preprocessor, default_providers, postprocess,
)

SRC_HW = (720, 1280)


def _lb_for(pre, hw=SRC_HW):
    _, lb = pre(np.zeros((hw[0], hw[1], 3), np.uint8))
    return lb


def _to_input(xyxy, lb):
    """Source-pixel box -> network-input box (inverse of the post mapping)."""
    b = np.asarray(xyxy, np.float64)
    return np.array([
        (b[0] - lb.roi_x) * lb.scale + lb.pad_x, (b[1] - lb.roi_y) * lb.scale + lb.pad_y,
        (b[2] - lb.roi_x) * lb.scale + lb.pad_x, (b[3] - lb.roi_y) * lb.scale + lb.pad_y,
    ])


def _raw(boxes_in, scores, nc=1):
    """Build a raw (1, 4 + nc, N) head from input-space xyxy boxes and (N, nc) scores."""
    b = np.asarray(boxes_in, np.float32).reshape(-1, 4)
    out = np.zeros((4 + nc, b.shape[0]), np.float32)
    out[0] = 0.5 * (b[:, 0] + b[:, 2])
    out[1] = 0.5 * (b[:, 1] + b[:, 3])
    out[2] = b[:, 2] - b[:, 0]
    out[3] = b[:, 3] - b[:, 1]
    out[4:] = np.asarray(scores, np.float32).reshape(b.shape[0], nc).T
    return out[None]


# Preprocessing


def test_letterbox_geometry_and_blob_layout():
    pre = Preprocessor((192, 320))
    img = np.zeros((720, 1280, 3), np.uint8)
    img[..., 0], img[..., 1], img[..., 2] = 10, 20, 30  # B, G, R
    blob, lb = pre(img)
    assert blob.shape == (1, 3, 192, 320) and blob.dtype == np.float32
    assert lb.scale == pytest.approx(0.25)
    assert (lb.pad_x, lb.pad_y) == (0.0, 6.0)
    # Content rows are RGB/255; padding rows are the letterbox grey
    assert blob[0, :, 100, 100] == pytest.approx(np.array([30, 20, 10]) / 255.0)
    assert blob[0, :, 0, 0] == pytest.approx(np.full(3, 114 / 255.0))
    assert blob[0, :, 191, 0] == pytest.approx(np.full(3, 114 / 255.0))


def test_blob_buffer_is_reused():
    pre = Preprocessor((192, 320))
    a, _ = pre(np.zeros((720, 1280, 3), np.uint8))
    b, _ = pre(np.full((720, 1280, 3), 255, np.uint8))
    assert a is b  # no per-frame allocation


def test_padding_refilled_when_layout_changes():
    pre = Preprocessor((192, 320))
    pre(np.full((720, 1280, 3), 255, np.uint8))  # pads top/bottom
    blob, lb = pre(np.full((400, 400, 3), 255, np.uint8))  # pads left/right
    assert lb.pad_y == 0.0 and lb.pad_x > 0
    assert blob[0, 0, 0, 0] == pytest.approx(114 / 255.0)


def test_roi_crop_offsets_and_out_of_frame_fallback():
    pre = Preprocessor((192, 320), roi=(0, 200, 1280, 600))
    lb = _lb_for(pre)
    assert (lb.roi_x, lb.roi_y) == (0, 200)
    assert lb.scale == pytest.approx(min(320 / 1280, 192 / 400))
    # ROI entirely outside a small frame: whole frame used, no crash
    lb_small = _lb_for(pre, (180, 320))
    assert (lb_small.roi_x, lb_small.roi_y) == (0, 0)
    assert lb_small.scale == pytest.approx(1.0)


# Postprocessing: raw head


def test_raw_round_trip_to_source_pixels():
    pre = Preprocessor((192, 320))
    lb = _lb_for(pre)
    src = [400.0, 300.0, 560.0, 420.0]
    det = postprocess(_raw([_to_input(src, lb)], [[0.9]]), lb, SRC_HW)
    assert len(det) == 1
    assert det.boxes[0] == pytest.approx(src, abs=1e-3)
    assert det.scores[0] == pytest.approx(0.9) and det.classes[0] == 0


def test_raw_round_trip_with_roi():
    pre = Preprocessor((192, 320), roi=(100, 200, 1180, 600))
    lb = _lb_for(pre)
    src = [400.0, 300.0, 560.0, 420.0]
    det = postprocess(_raw([_to_input(src, lb)], [[0.9]]), lb, SRC_HW)
    assert det.boxes[0] == pytest.approx(src, abs=1e-3)


def test_raw_threshold_and_class_agnostic_nms():
    lb = Letterbox(1.0, 0.0, 0.0, 0, 0)
    boxes = [[10, 10, 50, 50], [12, 11, 52, 49], [100, 100, 140, 140], [200, 10, 220, 30]]
    # box 1 overlaps box 0 but scores highest in a different class
    scores = [[0.8, 0.1], [0.1, 0.9], [0.7, 0.0], [0.2, 0.1]]
    det = postprocess(_raw(boxes, scores, nc=2), lb, (300, 300), conf=0.35, iou=0.5)
    assert len(det) == 2  # overlapping pair merged across classes, low-score box dropped
    order = np.argsort(-det.scores)
    assert det.classes[order].tolist() == [1, 0]
    assert det.scores[order] == pytest.approx([0.9, 0.7])


def test_raw_class_filter_reports_original_class_ids():
    lb = Letterbox(1.0, 0.0, 0.0, 0, 0)
    boxes = [[10, 10, 50, 50], [100, 100, 140, 140]]
    scores = [[0.9, 0.0, 0.0], [0.0, 0.0, 0.8]]
    det = postprocess(_raw(boxes, scores, nc=3), lb, (300, 300), class_ids=[2])
    assert len(det) == 1 and det.classes[0] == 2


def test_raw_nothing_above_threshold():
    lb = Letterbox(1.0, 0.0, 0.0, 0, 0)
    det = postprocess(_raw([[10, 10, 50, 50]], [[0.1]]), lb, (300, 300))
    assert len(det) == 0 and det.boxes.shape == (0, 4)


def test_boxes_clipped_to_image():
    lb = Letterbox(1.0, 0.0, 0.0, 0, 0)
    det = postprocess(_raw([[-20, -5, 350, 120]], [[0.9]]), lb, (100, 300))
    assert det.boxes[0].tolist() == [0.0, 0.0, 299.0, 99.0]


# Postprocessing: end-to-end head


def test_end_to_end_layout():
    lb = Letterbox(0.5, 0.0, 10.0, 0, 0)
    rows = np.array([
        [10, 20, 50, 60, 0.9, 0],
        [100, 20, 150, 60, 0.2, 0],  # below conf
        [60, 70, 90, 95, 0.6, 3],
        [0, 0, 0, 0, 0.0, 0],  # padding row
    ], np.float32)
    det = postprocess(rows[None], lb, SRC_HW, conf=0.35)
    assert len(det) == 2
    assert det.scores.tolist() == pytest.approx([0.9, 0.6])  # sorted by score
    assert det.boxes[0] == pytest.approx([20, 20, 100, 100])
    det3 = postprocess(rows[None], lb, SRC_HW, conf=0.35, class_ids=[3])
    assert len(det3) == 1 and det3.classes[0] == 3


def test_bad_output_shape_raises():
    with pytest.raises(ValueError):
        postprocess(np.zeros((1, 2, 3, 4)), Letterbox(1, 0, 0, 0, 0), SRC_HW)


def test_default_providers_orders_and_filters():
    got = [p for p, _ in default_providers(["CPUExecutionProvider", "CUDAExecutionProvider",
                                            "TensorrtExecutionProvider"])]
    assert got == ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]


# Runtime wrapper (needs onnxruntime + onnx; skipped in CI images without them)


def test_kart_detector_end_to_end_with_tiny_model(tmp_path):
    ort = pytest.importorskip("onnxruntime")  # noqa: F841
    onnx = pytest.importorskip("onnx")
    from onnx import TensorProto, helper, numpy_helper

    from autonomous_kart.nodes.opencv_pathfinder.kart_detector import KartDetector

    # Model ignores its pixels and always emits one raw-layout box in input
    # space: centred (160, 96), 80x40, score 0.9. Enough to test the wiring.
    head = np.zeros((1, 5, 4), np.float32)
    head[0, :, 0] = [160, 96, 80, 40, 0.9]
    const = numpy_helper.from_array(head, "head")
    zero = numpy_helper.from_array(np.zeros((1,), np.float32), "zero")
    graph = helper.make_graph(
        [
            helper.make_node("ReduceMean", ["images"], ["m"], keepdims=0),
            helper.make_node("Mul", ["m", "zero"], ["m0"]),  # ties output to input
            helper.make_node("Add", ["head", "m0"], ["output0"]),
        ],
        "tiny",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 192, 320])],
        [helper.make_tensor_value_info("output0", TensorProto.FLOAT, [1, 5, 4])],
        initializer=[const, zero],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    path = tmp_path / "tiny.onnx"
    onnx.save(model, str(path))

    det = KartDetector(str(path), providers=["CPUExecutionProvider"], warmup=1)
    assert det.input_hw == (192, 320)
    out = det(np.zeros((720, 1280, 3), np.uint8))
    # Input box (120, 76, 200, 116) -> source: /0.25 after removing pad_y=6
    assert len(out) == 1
    assert out.boxes[0] == pytest.approx([480, 280, 800, 440], abs=1e-3)
    assert set(det.last_timing_ms) == {"pre", "infer", "post", "total"}
