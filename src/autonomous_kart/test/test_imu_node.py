"""
Tests for ImuNode.

Focus is on the calibration state machine, motion guards, cmd_vel gating,
cache load/save, and the public ROS surface (`imu`, `imu/calibrate`,
`imu/calibration_status`). I2C reads are faked via a controllable stand-in
for `smbus2.SMBus`, and `publish_imu()` is mostly invoked directly so we
don't have to wait on the ROS timer to fire 200 times.
"""
import json
import math
import os
import sys
import time
import types

import pytest

rclpy_mod = pytest.importorskip("rclpy")

# smbus2 is a Pi-only runtime dep. Stub it before importing the node so the
# module loads on dev/CI machines where it isn't installed.
if "smbus2" not in sys.modules:
    _stub = types.ModuleType("smbus2")

    class _StubSMBus:
        def __init__(self, *_a, **_k): pass
        def write_byte_data(self, *_a, **_k): pass
        def read_i2c_block_data(self, *_a, **_k): return [0] * 14

    _stub.SMBus = _StubSMBus
    sys.modules["smbus2"] = _stub

from autonomous_kart.nodes.imu import imu_node as imu_module  # noqa: E402
from autonomous_kart.nodes.imu.imu_node import (  # noqa: E402
    CALIBRATED,
    CALIBRATING,
    ImuNode,
    WAITING,
)


# ----------------------------------------------------------------- helpers

def _to_be_s16(v: int):
    """Pack a signed 16-bit int into two big-endian unsigned bytes."""
    v = int(v) & 0xFFFF
    return [(v >> 8) & 0xFF, v & 0xFF]


def _make_burst(accel_raw=(0, 0, 16384), gyro_raw=(0, 0, 0)):
    """14-byte block: accel(6) + temp(2) + gyro(6), big-endian s16 each."""
    out = []
    for a in accel_raw:
        out += _to_be_s16(a)
    out += [0, 0]  # temp
    for g in gyro_raw:
        out += _to_be_s16(g)
    return out


class FakeBus:
    """In-memory replacement for SMBus. Tests can mutate `next_read`."""
    def __init__(self, *_a, **_k):
        self.next_read = _make_burst()
        self.writes = []

    def write_byte_data(self, addr, reg, val):
        self.writes.append((addr, reg, val))

    def read_i2c_block_data(self, _addr, _reg, _length):
        return list(self.next_read)


@pytest.fixture
def fake_bus(monkeypatch):
    bus_holder = {}

    def factory(*a, **k):
        bus = FakeBus(*a, **k)
        bus_holder["bus"] = bus
        return bus

    monkeypatch.setattr(imu_module, "SMBus", factory)
    return bus_holder  # caller pulls bus_holder["bus"] after node ctor


def _imu_params(cache_path, *, calib_samples=5, frequency=100.0):
    """Default parameter set for ImuNode tests. Small sample count keeps tests fast."""
    return {
        "simulation_mode": False,
        "i2c_bus": 1,
        "i2c_address": 0x68,
        "imu_frequency": frequency,
        "default_g": -9.80665,
        "accel_variance": 0.01,
        "gyro_variance": 0.001,
        "calibration_samples": calib_samples,
        "accel_per_g": 16384.0,
        "gyro_lsb_per_dps": 131.0,
        "gyro_motion_thresh": 0.05,
        "accel_motion_thresh": 0.5,
        "cmd_vel_zero_eps": 0.01,
        "cmd_vel_max_age_s": 1.0,
        "calibration_cache_path": str(cache_path),
        "status_frequency": 10.0,
    }


@pytest.fixture
def imu_factory(ros_ctx, fake_bus, tmp_path):
    """
    Returns a context-manager factory: `with imu_factory(**overrides) as (node, bus): ...`
    Builds an ImuNode with a fake SMBus and a temp cache path.
    """
    from contextlib import contextmanager

    @contextmanager
    def _build(cache_path=None, **overrides):
        path = cache_path or (tmp_path / "imu_calibration.json")
        params = _imu_params(path)
        params.update(overrides)
        with ros_ctx(params) as rclpy:
            node = ImuNode()
            try:
                yield node, fake_bus.get("bus"), rclpy
            finally:
                node.destroy_node()

    return _build


# ----------------------------------------------------------- cache loading


def test_missing_cache_starts_in_waiting(imu_factory, tmp_path):
    cache = tmp_path / "nope.json"
    with imu_factory(cache_path=cache) as (node, _bus, _rclpy):
        assert node.state == WAITING
        assert node._calib_count == 0


def test_valid_cache_starts_calibrated(imu_factory, tmp_path):
    cache = tmp_path / "cal.json"
    cache.write_text(json.dumps({
        "gyro_bias": [0.01, -0.02, 0.005],
        "samples": 200,
        "timestamp": 0.0,
    }))
    with imu_factory(cache_path=cache) as (node, _bus, _rclpy):
        assert node.state == CALIBRATED
        assert node.gyro_bias == pytest.approx([0.01, -0.02, 0.005])


def test_corrupt_cache_falls_through_to_waiting(imu_factory, tmp_path):
    cache = tmp_path / "cal.json"
    cache.write_text("{not valid json")
    with imu_factory(cache_path=cache) as (node, _bus, _rclpy):
        assert node.state == WAITING


def test_wrong_shape_cache_falls_through_to_waiting(imu_factory, tmp_path):
    cache = tmp_path / "cal.json"
    cache.write_text(json.dumps({"gyro_bias": [0.0, 0.0]}))  # length 2, invalid
    with imu_factory(cache_path=cache) as (node, _bus, _rclpy):
        assert node.state == WAITING


# ------------------------------------------------------------- cmd_vel gate


def test_calibration_blocked_when_cmd_vel_nonzero(imu_factory):
    with imu_factory(calibration_samples=5) as (node, bus, _rclpy):
        node._last_cmd_vel = 0.5
        node._last_cmd_vel_t = time.time()
        bus.next_read = _make_burst()  # still

        node.publish_imu()

        assert node.state == WAITING
        assert "non-zero" in node.last_error
        assert node._calib_count == 0


def test_calibration_blocked_when_cmd_vel_stale(imu_factory):
    with imu_factory() as (node, bus, _rclpy):
        node._last_cmd_vel = 0.0
        node._last_cmd_vel_t = time.time() - 5.0  # ancient
        bus.next_read = _make_burst()

        node.publish_imu()

        assert node.state == WAITING
        assert "stale" in node.last_error


def test_calibration_proceeds_when_no_cmd_vel_ever_received(imu_factory):
    """First-boot grace: never heard cmd_vel ⇒ assume idle and proceed."""
    with imu_factory() as (node, bus, _rclpy):
        assert node._last_cmd_vel is None
        bus.next_read = _make_burst()

        node.publish_imu()

        assert node.state == CALIBRATING
        assert node._calib_count == 1


# ------------------------------------------------------- motion abort guards


def test_calibration_aborts_on_gyro_motion(imu_factory):
    with imu_factory(calibration_samples=10) as (node, bus, _rclpy):
        # Build up a few good samples
        bus.next_read = _make_burst()
        for _ in range(3):
            node.publish_imu()
        assert node.state == CALIBRATING
        assert node._calib_count == 3

        # gyro raw 1000 / 131 * pi/180 ≈ 0.133 rad/s, well over 0.05 threshold
        bus.next_read = _make_burst(gyro_raw=(1000, 0, 0))
        node.publish_imu()

        assert node.state == WAITING
        assert "gyro x" in node.last_error
        assert node._calib_count == 0  # accumulator reset


def test_calibration_aborts_on_accel_anomaly(imu_factory):
    with imu_factory() as (node, bus, _rclpy):
        # Accel z = 0 means |accel| ~ 0, far from |g|. Gyro still.
        bus.next_read = _make_burst(accel_raw=(0, 0, 0))
        node.publish_imu()

        assert node.state == WAITING
        assert "|accel|" in node.last_error


# --------------------------------------------------- happy-path calibration


def test_calibration_completes_and_writes_cache(imu_factory, tmp_path):
    cache = tmp_path / "out.json"
    # Tiny non-zero gyro so the bias is testable, well below motion threshold.
    # 50 raw / 131 * pi/180 ≈ 0.00666 rad/s
    raw_gyro_x = 50
    expected_bias_x = raw_gyro_x / 131.0 * (math.pi / 180.0)

    with imu_factory(cache_path=cache, calibration_samples=8) as (node, bus, _rclpy):
        bus.next_read = _make_burst(gyro_raw=(raw_gyro_x, 0, 0))

        for _ in range(8):
            node.publish_imu()

        assert node.state == CALIBRATED
        assert node.gyro_bias[0] == pytest.approx(expected_bias_x, rel=1e-6)
        assert node.gyro_bias[1] == pytest.approx(0.0, abs=1e-9)
        assert node.gyro_bias[2] == pytest.approx(0.0, abs=1e-9)

        # Cache file written, atomic .tmp gone
        assert cache.exists()
        assert not (tmp_path / "out.json.tmp").exists()
        payload = json.loads(cache.read_text())
        assert payload["gyro_bias"] == pytest.approx(node.gyro_bias)
        assert payload["samples"] == 8
        assert "timestamp" in payload


def test_publish_emits_imu_only_after_calibration(imu_factory):
    """Drives the full WAITING → CALIBRATED arc and verifies imu publishes are gated."""
    from sensor_msgs.msg import Imu

    with imu_factory(calibration_samples=3) as (node, bus, rclpy):
        listener = rclpy.create_node("imu_listener")
        received = []
        listener.create_subscription(Imu, "imu", lambda m: received.append(m), 10)

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            # discovery
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                exe.spin_once(timeout_sec=0.05)

            bus.next_read = _make_burst()
            assert node.state == WAITING

            # While calibrating, no imu publishes should happen.
            for _ in range(3):
                node.publish_imu()
            assert node.state == CALIBRATED

            # Now a publish call should produce an Imu message.
            node.publish_imu()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and len(received) < 1:
                exe.spin_once(timeout_sec=0.05)

            assert len(received) >= 1
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()


def test_published_angular_velocity_subtracts_bias(imu_factory):
    """Once CALIBRATED, the published ω equals raw-gyro - bias."""
    from sensor_msgs.msg import Imu

    with imu_factory(calibration_samples=4) as (node, bus, rclpy):
        listener = rclpy.create_node("imu_listener")
        received = []
        listener.create_subscription(Imu, "imu", lambda m: received.append(m), 10)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                exe.spin_once(timeout_sec=0.05)

            # Calibrate with constant bias of raw_x=100 → bias_x stored
            bias_raw = 100
            bus.next_read = _make_burst(gyro_raw=(bias_raw, 0, 0))
            for _ in range(4):
                node.publish_imu()
            assert node.state == CALIBRATED

            # Now feed raw_x = 300; expect angular_velocity.x ≈ (300 - 100) raw worth
            bus.next_read = _make_burst(gyro_raw=(300, 0, 0))
            received.clear()
            node.publish_imu()

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and len(received) < 1:
                exe.spin_once(timeout_sec=0.05)
            assert len(received) == 1

            expected = (300 - bias_raw) / 131.0 * (math.pi / 180.0)
            assert received[0].angular_velocity.x == pytest.approx(expected, rel=1e-6)
            assert received[0].angular_velocity.y == pytest.approx(0.0, abs=1e-9)
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()


# --------------------------------------------------------- trigger / status


def test_calibrate_trigger_resets_to_waiting(imu_factory, tmp_path, spin_helper):
    """Sending an Empty on imu/calibrate while CALIBRATED resets the node."""
    from std_msgs.msg import Empty

    cache = tmp_path / "cal.json"
    cache.write_text(json.dumps({"gyro_bias": [0.0, 0.0, 0.0]}))

    with imu_factory(cache_path=cache) as (node, _bus, rclpy):
        assert node.state == CALIBRATED

        trigger = rclpy.create_node("trigger_pub")
        pub = trigger.create_publisher(Empty, "imu/calibrate", 1)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(trigger)
        try:
            spin_helper(exe, lambda: False, timeout=0.4)
            pub.publish(Empty())
            assert spin_helper(exe, lambda: node.state == WAITING, timeout=2.0)
            assert node.last_error == "triggered"
            assert node._calib_count == 0
        finally:
            exe.remove_node(trigger)
            exe.remove_node(node)
            trigger.destroy_node()


def test_cmd_vel_subscription_updates_internal_state(imu_factory, spin_helper):
    from std_msgs.msg import Float32

    with imu_factory() as (node, _bus, rclpy):
        pub_node = rclpy.create_node("vel_pub")
        pub = pub_node.create_publisher(Float32, "cmd_vel", 5)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(pub_node)
        try:
            spin_helper(exe, lambda: False, timeout=0.4)
            pub.publish(Float32(data=2.5))
            assert spin_helper(exe, lambda: node._last_cmd_vel == 2.5, timeout=2.0)
            assert node._last_cmd_vel_t > 0.0
        finally:
            exe.remove_node(pub_node)
            exe.remove_node(node)
            pub_node.destroy_node()


def test_status_payload_has_expected_fields(imu_factory):
    from std_msgs.msg import String

    with imu_factory() as (node, _bus, rclpy):
        listener = rclpy.create_node("status_listener")
        received = []
        listener.create_subscription(
            String, "imu/calibration_status", lambda m: received.append(m.data), 1
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            # Discovery + wait for the periodic status timer to fire at least once.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and len(received) < 1:
                exe.spin_once(timeout_sec=0.05)

            assert len(received) >= 1
            payload = json.loads(received[-1])
            assert set(payload.keys()) >= {
                "state", "samples", "target_samples",
                "gyro_bias", "last_error", "cache_path",
            }
            assert payload["state"] in (WAITING, CALIBRATING, CALIBRATED)
            assert payload["target_samples"] == node.calib_samples
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()


def test_save_cache_failure_is_logged_but_not_raised(imu_factory, tmp_path):
    """Pointing the cache at an unwritable location must not crash the calibration."""
    bad_path = tmp_path / "no_such_dir" / "nope.json"
    with imu_factory(cache_path=bad_path, calibration_samples=2) as (node, bus, _rclpy):
        bus.next_read = _make_burst()
        for _ in range(2):
            node.publish_imu()
        # Calibration succeeded in-memory even though the cache write failed.
        assert node.state == CALIBRATED
        assert not bad_path.exists()
